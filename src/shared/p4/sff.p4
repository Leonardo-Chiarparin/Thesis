/* -*- P4_16 -*- */
#include <core.p4>
#include <v1model.p4>

const bit<16> TYPE_IPV4 = 0x0800;

const bit<8>  PROTO_TCP = 6;
const bit<8>  PROTO_UDP = 17;

const bit<16> PORT_LUMINANCE = 5000;
const bit<16> PORT_CHROMINANCE = 5001;

const bit<19> CONGESTION_THRESHOLD = 50; // 25 -> 20 ms, 50 -> 40 ms

/*************************************************************************
*********************** H E A D E R S  ***********************************
*************************************************************************/

typedef bit<9>  egressSpec_t;
typedef bit<48> macAddr_t;
typedef bit<32> ip4Addr_t;

header ethernet_t {
    macAddr_t dstAddr;
    macAddr_t srcAddr;
    bit<16>   etherType;
}

header ipv4_t {
    bit<4>    version;
    bit<4>    ihl;
    bit<8>    diffserv;
    bit<16>   totalLen;
    bit<16>   identification;
    bit<3>    flags;
    bit<13>   fragOffset;
    bit<8>    ttl;
    bit<8>    protocol;
    bit<16>   hdrChecksum;
    ip4Addr_t srcAddr;
    ip4Addr_t dstAddr;
}

header tcp_t {
    bit<16> srcPort;
    bit<16> dstPort;
    bit<32> seqNo;
    bit<32> ackNo;
    bit<4>  dataOffset;
    bit<3>  res;
    bit<9>  flags;
    bit<16> window;
    bit<16> checksum;
    bit<16> urgentPtr;
}

header udp_t {
    bit<16> srcPort;
    bit<16> dstPort;
    bit<16> length;
    bit<16> checksum;
}

// Volumetric Point Cloud Compression ( V-PCC ) Header: 
// It acts as as a lightweight Service Function Chaining ( SFC ) identifier, 
// where "frame_type" may be either "0x01" ( Core / Geometry ) 
// or "0x02" ( Enhancement / Texture )
header vpcc_t {
    bit<8>  frame_type;
    bit<16> frame_id;
    bit<16> chunk_id;
    bit<16> total_chunks;
    bit<64> timestamp;
    bit<64> s_global;
    bit<64> center_x;
    bit<64> center_y;
    bit<64> center_z;
    bit<32> frame_length;
    bit<32> magic_word;
}

struct metadata_t {
    bit<8> is_vpcc_stream; // Flag to identify XR traffic
}

struct headers {
    ethernet_t ethernet;
    ipv4_t     ipv4;
    tcp_t      tcp;
    udp_t      udp;
    vpcc_t     vpcc;
}

/*************************************************************************
*********************** P A R S E R  ***********************************
*************************************************************************/

parser MyParser( packet_in packet, out headers hdr, inout metadata_t meta, inout standard_metadata_t standard_metadata ) {
    state start {
        transition parse_ethernet;
    }

    state parse_ethernet {
        packet.extract( hdr.ethernet );
        transition select( hdr.ethernet.etherType ) {
            TYPE_IPV4: parse_ipv4;
            default: accept;
        }
    }

    state parse_ipv4 {
        packet.extract( hdr.ipv4 );
        transition select( hdr.ipv4.protocol ) {
            PROTO_TCP: parse_tcp;
            PROTO_UDP: parse_udp;
            default: accept;
        }
    }

    state parse_tcp {
        packet.extract( hdr.tcp );
        transition accept;
    }

    state parse_udp {
        packet.extract( hdr.udp );
        transition select( hdr.udp.dstPort ) {
            PORT_LUMINANCE: parse_vpcc;
            PORT_CHROMINANCE: parse_vpcc;
            default: accept;
        }
    }

    state parse_vpcc {
        packet.extract( hdr.vpcc );
        meta.is_vpcc_stream = 1;
        transition accept;
    }
}

/*************************************************************************
************   C H E C K S U M    V E R I F I C A T I O N   *************
*************************************************************************/

control MyVerifyChecksum( inout headers hdr, inout metadata_t meta ) {
    apply {  }
}


/*************************************************************************
**************  I N G R E S S   P R O C E S S I N G   *******************
*************************************************************************/

control MyIngress( inout headers hdr, inout metadata_t meta, inout standard_metadata_t standard_metadata ) {
    action drop() {
        mark_to_drop( standard_metadata );
    }

    // Carrying return traffic via IPv4 
    action ipv4_forward( macAddr_t dstAddr, egressSpec_t port ) {
        standard_metadata.egress_spec = port;
        hdr.ethernet.srcAddr = hdr.ethernet.dstAddr;
        hdr.ethernet.dstAddr = dstAddr;
        hdr.ipv4.ttl = hdr.ipv4.ttl - 1;
    }

    table ipv4_lpm {
        key = { 
            hdr.ipv4.dstAddr: lpm; 
            }
        actions = { 
            ipv4_forward; 
            drop; 
            NoAction; 
        }
        size = 1024;
        default_action = NoAction();
    }

    apply {
        if ( hdr.ipv4.isValid() ) {
            ipv4_lpm.apply();
        }
    }
}

/*************************************************************************
****************  E G R E S S   P R O C E S S I N G   *******************
*************************************************************************/

control MyEgress( inout headers hdr, inout metadata_t meta, inout standard_metadata_t standard_metadata ) { 
    action drop() {
        mark_to_drop( standard_metadata );
    }

    register<bit<16>>( 64 ) corrupted_frame_register;
    register<bit<16>>( 64 ) expected_chunk_register;

    register<bit<32>>( 1 ) drop_chrominance_packets;
    register<bit<32>>( 1 ) queue_depth_register;

    apply { 
        bit<32> current_depth = ( bit<32> )standard_metadata.deq_qdepth;
        bit<32> max_depth;

        queue_depth_register.read( max_depth, 0 );

        if ( current_depth > max_depth ) {
            queue_depth_register.write( 0, current_depth );
        }

        if ( meta.is_vpcc_stream != 1 ) {
            return;
        }

        // If the packet does not belong to the "Enhancement" level ( Chroma Features / Texture ),
        if ( hdr.vpcc.frame_type != 2 ) {
            return;
        }

        // Packet Washing logic ( In-Network Adaptation )
        bit<16> corrupted_frame_id; // Note: It is necessary to provide a "Frame-Level Dropping" strategy

        bit<32> index = ( bit<32> )( hdr.vpcc.frame_id & 63 );
        bit<32> number;

        // BPP ( Big Packet Protocol, a frame-level commitment )
        if ( hdr.vpcc.chunk_id == 0 ) {
            if ( standard_metadata.enq_qdepth > CONGESTION_THRESHOLD ) {
                corrupted_frame_register.write( index, hdr.vpcc.frame_id );
            } 
            
            else {
                corrupted_frame_register.write( index, 65535 );
                expected_chunk_register.write( index, 1 );
            }
        }
        
        else {
            corrupted_frame_register.read( corrupted_frame_id, index );

            if ( hdr.vpcc.frame_id != corrupted_frame_id ) {
                bit<16> expected_chunk;

                expected_chunk_register.read( expected_chunk, index );

                if ( hdr.vpcc.chunk_id != expected_chunk ) {
                    corrupted_frame_register.write( index, hdr.vpcc.frame_id );
                } 
                
                else {
                    expected_chunk_register.write( index, expected_chunk + 1 );
                }
            }
        }

        corrupted_frame_register.read( corrupted_frame_id, index );

        if ( hdr.vpcc.frame_id == corrupted_frame_id ) {
            drop_chrominance_packets.read( number, 0 );
            number = number + 1;             
            drop_chrominance_packets.write( 0, number );

            drop(); // Note: The frame is finally cancelled, hence performing an Active Queue Management ( AQM-like ) trimming
        }
    }
}

/*************************************************************************
*************   C H E C K S U M    C O M P U T A T I O N   **************
*************************************************************************/

control MyComputeChecksum( inout headers hdr, inout metadata_t meta ) {
    apply {
        update_checksum( hdr.ipv4.isValid(), { hdr.ipv4.version, hdr.ipv4.ihl, hdr.ipv4.diffserv, hdr.ipv4.totalLen, hdr.ipv4.identification, hdr.ipv4.flags, hdr.ipv4.fragOffset, hdr.ipv4.ttl, hdr.ipv4.protocol, hdr.ipv4.srcAddr, hdr.ipv4.dstAddr }, hdr.ipv4.hdrChecksum, HashAlgorithm.csum16 );
    }
}

/*************************************************************************
***********************  D E P A R S E R  *******************************
*************************************************************************/

control MyDeparser( packet_out packet, in headers hdr ) {
    apply {
        packet.emit( hdr.ethernet );
        packet.emit( hdr.ipv4 );
        packet.emit( hdr.tcp );
        packet.emit( hdr.udp );
        packet.emit( hdr.vpcc );
    }
}

/*************************************************************************
***********************  S W I T C H  *******************************
*************************************************************************/

V1Switch(
    MyParser(),
    MyVerifyChecksum(),
    MyIngress(),
    MyEgress(),
    MyComputeChecksum(),
    MyDeparser()
) main;