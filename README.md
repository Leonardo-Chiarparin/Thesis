# 🌐 DPDK-Based Service Function Chaining for Real-Time Point-Cloud Streaming

> **Experimental Thesis Project — Sapienza University of Rome**  
> A data-plane-oriented architecture for real-time volumetric point-cloud transport, in-network geometric aggregation, GPU projection, and hardware-accelerated H.265 delivery.

### 👥 Academic Information

**Author:** Leonardo Chiarparin ( Student ID: **2016363** )  
**Thesis Supervisor:** Professor Marco Polverini  
**Degree Programme:** Engineering in Computer Science  
**Institution:** **Sapienza University of Rome**

---

## 🧭 Current Implementation Status

This repository is an experimental research platform rather than a production-ready Service Function Chaining framework. The current snapshot implements and evaluates the upstream portion of the intended volumetric chain, while the downstream reconstruction and user-facing stages are still under development.

| Component | Current Status | Principal Responsibility |
|---|---|---|
| `Camera` | Implemented | DPDK-native point-cloud source, frame packetisation, pacing, and source telemetry |
| `SFF1` | Implemented | Temporal filtering, packet-level geometric aggregation, and NSH-style service metadata insertion |
| `SFF2` Route 0 | Implemented | `SFF1 -> Encoder` service forwarding |
| `Encoder` | Implemented | Frame assembly, centroid/radius processing, CUDA projection, persistent NVENC encoding, MPEG-TS framing, and output packetisation |
| `SFF2` Route 1 | Implemented | `Encoder -> Decoder` compressed-media forwarding |
| `Decoder` | Under development | H.265 decode and geometric reconstruction |
| `SFF2` Route 2 | Reserved, not operational | Intended `Decoder -> SFF3` forwarding; the Decoder-side packet format is still undefined in the current `SFF2` implementation |
| `SFF3` | Under development | Final service-function stage and downstream feedback entry point |
| `User` | Under development | Rendering, interaction, and final user-perceived QoE measurements |

The **currently implemented upstream chain** is represented horizontally as:

```text
Camera -> SFF1 -> SFF2 ( Route 0 ) -> Encoder -> SFF2 ( Route 1 )
```

The **complete target primary chain** is:

```text
Camera -> SFF1 -> SFF2 ( R0 ) -> Encoder -> SFF2 ( R1 ) -> Decoder -> SFF2 ( R2 ) -> SFF3 -> User
```

The intended reverse control path is:

```text
User -> SFF3 -> SFF2 -> Encoder -> SFF2 -> SFF1 -> Camera
```

However, in the current code, the implemented feedback path effectively terminates at `SFF1`: the Encoder consumes pose / zoom / temporal-control values and forwards the control packet to `SFF1`, while `SFF1` consumes the temporal skip. The current Camera implementation does **not** parse or apply feedback packets.

> **Repository snapshot note:** the previously discussed cross-node `pipeline/` analysis directory and its associated Python analysis script have been **temporarily removed**. This README therefore documents the native node telemetry exported by `Camera`, `SFF1`, `SFF2`, and `Encoder`, but it does not advertise a cross-node plotting script that is not presently part of the repository.

---

## 🎯 1. Project Motivation and Research Objective

The project investigates whether selected functions of a real-time volumetric streaming pipeline can be moved away from conventional application-level microservices and executed directly while packets traverse a DPDK-based Service Function Chain.

The objective is **not** simply to replace kernel sockets with a faster packet-I/O API. The central research question is whether the forwarding path can become an active computational component without overloading the data plane or compromising the semantic correctness of frame-level processing.

The design therefore separates operations according to their mathematical structure.

Operations that are **associative, incrementally composable, and naturally packet-local** are candidates for in-network execution:

```text
sum_x, sum_y, sum_z
min_x, min_y, min_z
max_x, max_y, max_z
active_point_count
original_point_count
```

Operations that require the complete frame, the final centroid, or GPU visibility decisions remain in the Encoder:

```text
centroid-dependent maximum-radius search
final geometric scaling
pose transformation
transformed bounding-box evaluation
six-view projection
visibility / depth conflict resolution
geometry / texture / occupancy atlas packing
I420 generation
H.265 / NVENC compression
```

This division is fundamental. It avoids turning `SFF1` into a general-purpose frame processor while still removing a meaningful class of reductions from the Encoder.

### Relation to the Reference Application-Level Pipeline

The experimental approach is informed by relevant state-of-art literature, such as the thesis *Point Cloud Coding for Extended Reality Services* by *Maria Giovanna Lacaria*, which describes a complete application-level volumetric streaming platform and establishes a detailed methodology for performance evaluation.

The present project does **not** reproduce that architecture verbatim. Instead, it reformulates the same workload around:

```text
DPDK-native packet I/O
OVS-DPDK switching
explicit service chaining
NSH-style SPI / SI steering
in-network geometric aggregation
GPU projection
persistent FFmpeg / NVENC encoding
frame-aware MPEG-TS packetisation
per-node telemetry
```

Consequently, comparisons with the reference implementation must be made only at **semantically equivalent boundaries**. Exact numerical equality is neither expected nor methodologically correct when the transport mechanisms, buffering behaviour, CPU placement, cache mode, or pacing policy differ.

---

## 🧩 2. Architectural Roles and Why Each Node Matters

| Node | Why It Is Architecturally Important |
|---|---|
| `Camera` | Establishes the source schedule, converts each binary frame into bounded MTU-safe DPDK packets, and controls the temporal shape of the offered load through explicit burst pacing. Without a stable source, downstream latency and retry measurements are not interpretable. |
| `SFF1` | Demonstrates the principal thesis contribution: useful geometric work is performed while point packets are already crossing the data plane. It also implements temporal filtering, making it both a computational and traffic-reduction function. |
| `SFF2` | Separates service steering from service computation. A single four-port forwarder enforces the primary and reverse service paths, decrements the Service Index, rewrites the next-hop network envelope, isolates route-specific telemetry, and optionally applies a congestion heuristic. |
| `Encoder` | Reconstructs complete frames, consumes the cumulative metadata produced by `SFF1`, executes the non-associative geometric stages, protects DPDK reception during long CPU / GPU work, generates the multi-view representation, and transforms it into frame-attributed MPEG-TS traffic. |

`OVS-DPDK` remains deliberately simple. It provides deterministic adjacency between vhost-user interfaces; it does not perform point-cloud computations. The service functions are therefore the elements responsible for application-specific data-plane work.

---

## 🔗 3. Service-Chain Semantics

### 3.1 Primary Service Path

The project uses:

```text
PRIMARY_SPI = 100
```

The implemented primary transitions are:

```text
SFF1 output:                 SPI 100, SI 255
SFF2 Route 0:  SI 255 -> 254, forwarded to Encoder
Encoder output:              SPI 100, SI 253
SFF2 Route 1:  SI 253 -> 252, forwarded toward Decoder
Future Decoder output:       expected SPI 100, SI 251
SFF2 Route 2:                reserved for SI 251 -> 250 toward SFF3
```

The complete intended chain is therefore:

```text
Camera -> SFF1 [ 100/255 ] -> SFF2 -> Encoder [ receives 100/254 ] -> Encoder emits [ 100/253 ] -> SFF2 -> Decoder [ receives 100/252 ] -> ...
```

`SFF2` Route 2 currently contains an explicit `TO BE DEFINED` branch for Decoder-side parsing. Although the route identifier and port mapping are reserved, Route 2 must **not** be described as implemented or validated until the Decoder packet format is defined.

### 3.2 Feedback Service Path

The project reserves:

```text
FEEDBACK_SPI = 200
```

The implemented steering logic is:

```text
SFF3 -> SFF2:      SI 255 -> 254 -> Encoder
Encoder consumes:  skip, yaw, pitch, zoom
Encoder -> SFF2:   Encoder rewrites SI to 253
SFF2 -> SFF1:      SI 253 -> 252 -> SFF1
SFF1 consumes:     temporal skip
```

The current Camera does not consume the returned command. Therefore, the presently implemented control semantics are distributed between the Encoder and `SFF1`, rather than being completed by a final Camera-side control operation.

### 3.3 Important Protocol-Scope Clarification

The project uses an 8-byte structure named `nsh_hdr` and adopts the **SPI / SI concept** of Network Service Header steering. Nevertheless, the current wire representation should be described as **NSH-style / NSH-inspired**, not as a fully RFC 8300-compliant MD-Type-2 implementation.

The reason is structural:

- `md_type = 0x02` is configured;
- a fixed project-specific 56-byte `int_hdr` is placed directly after the 8-byte steering header;
- that 56-byte block is **not encoded as RFC 8300 MD-Type-2 TLVs**;
- `next_protocol = 0x03` is retained even though the bytes following the steering header are project metadata and Camera metadata rather than an immediately nested IPv4 header;
- the current `base_flags_ttl_len` constant is part of the project's closed experimental format and should not be presented as a standards-conformant description of the complete custom metadata envelope.

This choice is acceptable for a controlled experiment because every participating node uses the same parser, but interoperability with a generic RFC 8300 implementation is **not** claimed.

The UDP port used by the service-chain nodes is likewise best described as a **project-selected service-chain port**:

```text
UDP_PORT = 6633
```

rather than as a generic standard NSH-over-UDP port.

---

## 📦 4. Data Representation and Packet Formats

### 4.1 Endianness and Portability

The current experiment runs on a homogeneous host and intentionally avoids unnecessary conversion of floating-point fields on the hot path.

Accordingly:

- integer identifiers and counters are converted with `htonl`, `htons`, or the corresponding DPDK helpers where implemented;
- point coordinates, geometric floating-point metadata, pose values, and several Encoder floating-point fields remain in native host representation;
- the offline converter explicitly writes little-endian `float32` point coordinates.

The resulting binary / wire format is therefore appropriate for the current homogeneous little-endian environment, but it is **not a machine-independent serialisation format**. A heterogeneous deployment would require explicit floating-point serialisation.

### 4.2 Common Project Structures

| Structure | Size | Function |
|---|---:|---|
| `point_tx` | 16 B | `x`, `y`, `z` as `float32`, RGB as `uint8`, plus one explicit padding byte |
| `cam_hdr` | 40 B | Frame identity, packet sequence, Camera timestamp, pose, temporal skip, original point count, points in current packet |
| `nsh_hdr` | 8 B | Project steering header carrying SPI / SI semantics |
| `int_hdr` | 56 B | Cumulative sums, minima, maxima, active-point count, original-point count |
| `enc_hdr` | 48 B | Encoded-frame identity, per-frame media chunk ID, scale and reconstruction metadata |
| `feedback_payload` | 14 B | Temporal skip, yaw, pitch, zoom |

### 4.3 Point Record — 16 Bytes

```text
+--------------------+ 4 B
| x : float32        |
+--------------------+ 4 B
| y : float32        |
+--------------------+ 4 B
| z : float32        |
+--------------------+ 1 B
| r : uint8          |
+--------------------+ 1 B
| g : uint8          |
+--------------------+ 1 B
| b : uint8          |
+--------------------+ 1 B
| padding : uint8    |
+--------------------+
Total = 16 B
```

The explicit padding byte creates a deterministic 16-byte record that is shared by the converter and the DPDK Camera. The important property is the stable binary layout; the existence of a 16-byte record should not itself be described as proof of SIMD execution.

### 4.4 Camera Header

The 40-byte Camera header carries the frame context that must survive the chain:

```text
frame_id
sequence_number
camera timestamp
yaw
pitch
zoom
temporal_skip
padding
original_points
points_in_packet
```

`frame_id` identifies the original source frame. `sequence_number` identifies the Camera packet within that frame. `original_points` defines the expected complete frame size, while `points_in_packet` allows each receiver to reconstruct point counts without deriving them from an external index.

### 4.5 INT Metadata Produced by SFF1

The 56-byte project INT block contains:

```text
sum_x, sum_y, sum_z
min_x, min_y, min_z
max_x, max_y, max_z
active_point_count
original_point_count
```

The values are **cumulative snapshots**, not per-packet reductions. As `SFF1` scans the frame, each forwarded packet carries the aggregation state available at that moment. The Encoder therefore retains the snapshot with the greatest `active_point_count`, which represents the most complete aggregation state observed for that frame.

### 4.6 Encoder Metadata

The 48-byte Encoder header contains:

```text
packet_id
frame_id
global_scale
box_center_x, box_center_y, box_center_z
final_scale
yaw, pitch
centroid_x, centroid_y, centroid_z
```

The semantics are deliberately frame-aware:

```text
enc_hdr.frame_id  = original point-cloud / video frame ID
enc_hdr.packet_id = DPDK MPEG-TS chunk index within that encoded frame
```

`packet_id` restarts from `0` when a new video PES is associated with a new encoded frame. It is **not** a global stream-wide packet counter.

---

## 📐 5. MTU-Aware Packet Design

The project designs every normal packet around a standard:

```text
IPv4 MTU = 1500 B
```

The calculations below treat the IPv4 MTU correctly as the maximum IPv4 packet size. Ethernet FCS, preamble, and inter-frame gap are not included.

### 5.1 Camera -> SFF1

The Camera uses:

```text
POINTS_PER_PACKET = 80
POINT_SIZE_BYTES  = 16
```

Therefore:

```text
Point payload = 80 x 16 = 1280 B
```

Layout:

```text
[ Ethernet 14 B ][ IPv4 20 B ][ UDP 8 B ][ CamHdr 40 B ][ Points <= 1280 B ]
```

Maximum size:

```text
IPv4 packet = 20 + 8 + 40 + 1280 = 1348 B
L2 frame    = 14 + 1348           = 1362 B
IPv4 margin = 1500 - 1348         = 152 B
```

The value `80` is intentionally conservative because the same point packet will later receive the additional service metadata inserted by `SFF1`.

### 5.2 SFF1 -> SFF2 -> Encoder

`SFF1` removes the previous Ethernet / IPv4 / UDP envelope, preserves the Camera header and point payload inside the existing `mbuf`, and prepends a new envelope containing the project steering and INT blocks.

Layout:

```text
[ Ethernet 14 B ][ IPv4 20 B ][ UDP 8 B ][ NSH-style 8 B ][ INT 56 B ][ CamHdr 40 B ][ Points <= 1280 B ]
```

Maximum size:

```text
IPv4 packet = 20 + 8 + 8 + 56 + 40 + 1280 = 1412 B
L2 frame    = 14 + 1412                   = 1426 B
IPv4 margin = 1500 - 1412                 = 88 B
```

The theoretical maximum number of complete 16-byte points under this final envelope would be:

```text
floor( ( 1500 - 20 - 8 - 8 - 56 - 40 ) / 16 ) = 85 points
```

The selected value of `80` therefore retains a useful margin.

> The forwarding operation should not be called universally "zero-copy". `SFF1` reuses the received `mbuf` and preserves the payload in place when replacing headers, which avoids a full payload copy at that stage; the Camera itself still copies point bytes into newly allocated `mbuf` storage.

### 5.3 Encoder -> SFF2 -> Decoder

The Encoder transports compressed MPEG-TS rather than raw points:

```text
[ Ethernet 14 B ][ IPv4 20 B ][ UDP 8 B ][ NSH-style 8 B ][ CamHdr 40 B ][ EncHdr 48 B ][ MPEG-TS <= 1316 B ]
```

MPEG-TS packets are fixed at:

```text
TS_PACKET_SIZE = 188 B
```

The DPDK media payload is intentionally:

```text
MTU_PAYLOAD_SIZE = 7 x 188 = 1316 B
```

Maximum packet size:

```text
IPv4 packet = 20 + 8 + 8 + 40 + 48 + 1316 = 1440 B
L2 frame    = 14 + 1440                   = 1454 B
IPv4 margin = 1500 - 1440                 = 60 B
```

The maximum media space available after the project headers is:

```text
1500 - ( 20 + 8 + 8 + 40 + 48 ) = 1376 B
```

Hence:

```text
7 x 188 = 1316 B  -> valid
8 x 188 = 1504 B  -> exceeds the available 1376 B
```

Seven TS packets are therefore the largest complete MPEG-TS group that fits without IP fragmentation.

### 5.4 Feedback Packet

The control packet is intentionally small:

```text
[ Ethernet 14 B ][ IPv4 20 B ][ UDP 8 B ][ NSH-style 8 B ][ Feedback 14 B ]
```

Result:

```text
IPv4 packet = 20 + 8 + 8 + 14 = 50 B
L2 frame    = 14 + 50         = 64 B
```

---

## 📷 6. Camera — Source Scheduling, Pacing, and Telemetry

### 6.1 Role

The Camera is the only node that originates the volumetric schedule. It reads the pre-converted Loot sequence, assigns source frame IDs, packetises each point cloud, timestamps the frame, and submits DPDK bursts toward `SFF1`.

The reference workload is configured as:

```text
K_FRAMES   = 300
TARGET_FPS = 30.0
BURST_SIZE = 32
```

### 6.2 Input Modes

Three cache / storage modes are implemented:

| Mode | Allocation | `fread()` | Meaning |
|---|---|---|---|
| `CACHE_MODE_BEST` | Before streaming | Before streaming | Cleanest datapath-oriented experiment; frame bytes are already resident in the application buffer |
| `CACHE_MODE_MIDDLE` | Before streaming | Inside frame loop | Storage-aware experiment; disk / page-cache access remains visible in Camera residency |
| `CACHE_MODE_WORST` | Inside frame loop | Inside frame loop | Deliberately pessimistic mode including allocation and file read in the frame path |

The current source selects:

```c
#define CACHE_MODE CACHE_MODE_MIDDLE
```

Results obtained with different cache modes are different experimental conditions and must not be mixed in the same performance claim.

### 6.3 Packetisation

For a frame containing `N` points:

```text
packets_per_frame = ceil( N / 80 )
bursts_per_frame  = ceil( packets_per_frame / 32 )
```

Each packet receives the same frame timestamp (`t_send_start`) and a monotonically increasing Camera `sequence_number`.

### 6.4 Why Pacing Is Fundamental

The current Camera explicitly enables:

```text
PACING_MODE   = 1
PACING_MARGIN = 100 %
```

Without pacing, one high-resolution point-cloud frame can produce thousands of packets in a very short CPU interval. A DPDK producer can therefore fill its TX ring faster than `OVS-DPDK` and `SFF1` can drain and process the corresponding traffic, even when no later service function is active.

This explains why `tx_retries` can become non-zero with only:

```text
Camera -> SFF1
```

present. The Camera does not need the complete downstream chain to experience backpressure: the local virtio-user ring, the OVS PMD, and the immediately connected receiver already form a finite producer-consumer system.

The implemented pacing window is derived from the remaining time before the nominal end of the current source period:

```text
target_end_time  = sequence_start + ( frame_index + 1 ) * frame_period
send_window      = max( 0, target_end_time - t_send_start )
send_window      = send_window * PACING_MARGIN / 100
burst_interval   = send_window / total_bursts
```

Before each full or final burst, the Camera busy-waits until the next planned burst instant and records that deliberate wait in `frame_pacing_cycles`.

With `PACING_MARGIN = 100`, the available transmission window is distributed across the complete remaining frame period. If the Camera is already late because disk I/O or another operation has consumed the period, then:

```text
send_window = 0
```

and the current implementation cannot create pacing time for that frame; the source then transmits as quickly as the TX path allows. Therefore, `PACING_MODE = 1` means that pacing is **available and applied when a positive scheduling budget exists**, not that every frame necessarily contains a non-zero pacing delay.

### 6.5 Meaning of `tx_retries`

`tx_retries` is **not a UDP retransmission counter** and it is not evidence that a packet was resent after network loss.

The counter is incremented only when:

```text
rte_eth_tx_burst( ... ) == 0
```

meaning that the DPDK TX queue accepted no packet from the current pending burst at that instant. The application then executes an increasing `rte_pause()` backoff and retries the same unsent `mbuf` pointers.

If some packets are accepted ( `nb_tx > 0` ), progress is recorded and the consecutive retry counter is reset. If the maximum retry limit is exceeded, remaining unsent `mbuf`s are freed.

Explicit pacing is therefore important because it reduces short-duration queue pressure **before** the retry path becomes necessary.

The difference from an application-level TCP implementation is equally important. A blocking `send()` / `sendall()` path is mediated by kernel socket buffers, TCP flow control, congestion control, and scheduler blocking. DPDK bypasses most of that machinery; the application itself becomes responsible for how aggressively it offers packets to the ring. Consequently, the fact that the reference application did not require an explicit pacing loop does not imply that the DPDK source should behave identically without one.

### 6.6 Camera Telemetry — Exact Semantics

The Camera records telemetry in memory and exports it after the sequence, avoiding CSV writes in the hot path.

For each frame:

**Disk I/O**

```text
disk_io_ms = t_io_end - t_io_start
```

It is non-zero only when the selected cache mode performs file I/O inside the frame loop.

**TX wall-time excluding deliberate pacing**

```text
raw_send_cycles = t_send_end - t_send_start
tx_cycles       = raw_send_cycles - frame_pacing_cycles
tx_duration_ms  = tx_cycles / timer_hz * 1000
```

`tx_duration_ms` still includes non-pacing transmission overhead, packet-submission stalls, and retry / backoff time. It intentionally excludes only the deliberate source-shaping waits.

**Active TX API time**

```text
active_tx_ms = sum( duration of each rte_eth_tx_burst call )
```

This is a strict subset of the broader TX wall-time. It does not include pacing, packet construction, or `rte_pause()` retry backoff.

**Logical payload**

```text
payload_bytes = tx_points * 16
logical_frame_bytes = payload_bytes + 40 B   if at least one packet was transmitted
```

The Camera's telemetry deliberately counts the Camera header once per logical frame for throughput / bitrate semantics. It does **not** represent true on-wire byte consumption, where the Camera header, UDP header, IPv4 header, and Ethernet header are repeated for every packet.

**Internal throughput**

```text
internal_throughput_mbs = ( logical_frame_bytes / 1,000,000 ) / tx_duration_seconds
```

This is a pacing-excluded logical application throughput, expressed in decimal MB/s.

**Logical network bitrate**

```text
network_bitrate_mbps = logical_frame_bytes * 8 * TARGET_FPS / 1,000,000
```

Again, this is a logical point-cloud bitrate, not an Ethernet wire-rate measurement.

**Residency**

```text
total_residency_ms = t_send_end - t_start_residency
```

The interval begins before the per-frame disk operation and ends when Camera transmission finishes. It includes disk work, packet preparation, retries, and deliberate intra-frame pacing. The subsequent inter-frame schedule wait occurs after telemetry finalisation and is not included.

**Camera efficiency**

```text
node_efficiency_pct = ( disk_io_seconds + tx_duration_seconds ) / residency_seconds * 100
```

Because `tx_duration_seconds` excludes deliberate pacing while `residency_seconds` includes it, active pacing correctly appears as non-productive residency in this metric. `active_tx_ms` is intentionally **not** the numerator of Camera efficiency.

**Reliability indicators**

```text
tx_retries       = number of zero-accept TX-burst events
mbuf_starvation  = number of frame-level mbuf allocation failures recorded by the current Camera path
```

### 6.7 End-of-Stream Behaviour

The reserved sentinel is:

```text
frame_id = 0xFFFFFFFF
```

After the sequence, the Camera emits multiple EOS copies ( up to `BURST_SIZE` instances ) to improve the probability that downstream stages observe termination in the UDP / DPDK environment. EOS is protocol signalling, not a normal data frame, and is excluded from ordinary frame telemetry.

---

## 🧠 7. SFF1 — Temporal Filtering and In-Network Geometry

### 7.1 Role

`SFF1` is the main computational offloading stage of the present design. It has two DPDK ports:

```text
PORT_RX = Camera-facing ingress
PORT_TX = SFF2-facing egress and feedback ingress
```

It receives every Camera frame, optionally filters the frame according to the active temporal skip, computes cumulative geometric reductions for frames that remain active, and forwards those packets with the project steering + INT envelope.

### 7.2 Temporal Skip

`SFF1` selects the active skip as:

```text
active_skip = dynamic_feedback_skip     if dynamic_feedback_skip > 0
active_skip = Camera temporal_skip      otherwise
```

A value of `0` in the dynamic controller means "fall back to the Camera-provided skip" rather than "drop every frame".

The frame forwarding decision is:

```text
status = 1  if ( frame_id - 1 ) mod active_skip == 0
status = 0  otherwise
```

A skipped frame is still received and accounted for at `SFF1`, but its point packets are freed locally instead of being forwarded. This is what makes temporal skipping a genuine traffic-reduction operation in the data plane.

### 7.3 Incremental Geometric Offloading

For every point belonging to an active frame, `SFF1` updates:

```text
sum_x += x
sum_y += y
sum_z += z

min_axis = min( min_axis, coordinate )
max_axis = max( max_axis, coordinate )

active_point_count++
```

The operation is valid in-network because sums, extrema, and counters can be incrementally accumulated without requiring the complete frame in memory.

Each outgoing packet carries the **current cumulative snapshot**. The final packet of a complete frame therefore normally carries the complete sums / extrema / count state, but the Encoder is written defensively and selects the received snapshot with the greatest `active_point_count` rather than assuming arrival order alone proves completeness.

### 7.4 In-Place Header Replacement

`SFF1` strips the previous Ethernet / IPv4 / UDP envelope from the existing `mbuf`, preserves:

```text
CamHdr + point payload
```

and prepends:

```text
new Ethernet + IPv4 + UDP + NSH-style header + INT block
```

The point payload is therefore not copied into a second application frame buffer at `SFF1`. This is one of the reasons the aggregation can be performed efficiently in the forwarding path.

### 7.5 Feedback Handling

`SFF1` also polls its SFF2-facing port for feedback. It accepts the project feedback service path when the packet reaches the expected feedback SPI / SI and extracts the requested temporal skip.

The control packet is consumed by `SFF1` after the update. The current code does not forward that packet onward to the Camera.

### 7.6 SFF1 Telemetry — Exact Semantics

The first packet of a new frame is timestamped immediately after its Camera header is identified. This prevents finalisation work for the previous frame from contaminating the new frame's arrival timestamp.

**TX duration**

```text
tx_duration_ms = last_successful_TX - first_successful_TX
```

It is defined only for forwarded frames for which successful TX timestamps exist.

**Active processing**

```text
active_process_ms = accumulated time spent in packet processing and forwarding work
```

This includes the measured processing windows around skip handling, geometric accumulation, header manipulation, and flush activity. It is not identical to residency.

**Frame residency**

```text
total_residency_ms = frame_end - first_frame_arrival
```

`frame_end` follows the implementation precedence: `frame_completion_cycles` when available; otherwise `frame_last_activity_cycles`; otherwise the current finalisation timestamp. For a successfully forwarded complete frame, `frame_completion_cycles` is normally set to the last successful TX timestamp, whereas skipped or error paths can complete on the last measured activity.

**Cycle and header wait**

```text
cycle_ms       = frame_end - previous_cycle_end
header_wait_ms = max( 0, cycle_ms - total_residency_ms )
```

`header_wait_ms` is therefore an **inter-frame idle component**. It does not belong to the current frame's residency.

**Node efficiency**

```text
node_efficiency_pct = total_residency_ms / cycle_ms * 100
```

For `SFF1`, efficiency is a **cycle occupancy / duty-cycle metric**, aligned with the reference flow-controller semantics. It is not computed as `active_process_ms / total_residency_ms`.

**Logical throughput**

```text
logical_payload_bytes = rx_points * 16
logical_frame_bytes   = logical_payload_bytes + 40 B   if at least one packet was received

internal_throughput_mbs = ( logical_frame_bytes / 1,000,000 ) / residency_seconds
```

This metric characterises the complete frame residency of the relay / processing node; it is not a pure NIC receive-rate measurement.

**Effective bitrate**

```text
network_bitrate_mbps = logical_frame_bytes * 8 * ( TARGET_FPS / temporal_skip ) / 1,000,000
```

For performance analysis, `status = 1` frames are the meaningful forwarded-frame population when comparing the effective output path.

**Camera-to-SFF1 arrival latency**

```text
camera_to_node_latency_ms = first_SFF1_arrival - Camera_frame_TX_timestamp
```

This calculation assumes the participating DPDK lcores share the same host timer domain, which is true for the current single-host experiment.

**Schedule delay**

```text
real_elapsed  = frame_end - first_session_arrival
ideal_elapsed = ( frame_id - first_arrival_frame_id ) / TARGET_FPS
schedule_delay_ms = ( real_elapsed - ideal_elapsed ) * 1000
```

**Arrival jitter**

Because `SFF1` receives every Camera frame before applying its own temporal filter:

```text
network_jitter_ms = abs( real_interarrival - 1 / TARGET_FPS ) * 1000
```

**TX retries** retain the DPDK meaning used by the Camera: they count zero-accept TX-burst events, not transport-layer retransmissions.

---

## 🔀 8. SFF2 — Multi-Port Service Forwarding and Congestion Heuristic

### 8.1 Role and Ports

`SFF2` is a four-port DPDK forwarding function:

| DPDK Port | Adjacent Function |
|---:|---|
| `0` | `SFF1` |
| `1` | `Encoder` |
| `2` | `Decoder` |
| `3` | `SFF3` |

Its importance is architectural: computation remains in the service functions, while `SFF2` centralises service-path steering, next-hop envelope rewriting, route-specific telemetry, and the optional data-plane congestion experiment.

### 8.2 Implemented Primary Routing

```text
Route 0 : SFF1    -> SFF2 -> Encoder
Route 1 : Encoder -> SFF2 -> Decoder
Route 2 : Decoder -> SFF2 -> SFF3   [ reserved, parser not yet defined ]
```

The primary classifier is based on both ingress port and expected SPI / SI. This prevents an otherwise valid-looking SI from being accepted from an unintended side of the topology.

For implemented primary packets, `SFF2`:

1. validates Ethernet, IPv4, UDP, and the project steering header;
2. selects the expected egress port;
3. derives the route-specific frame identifier;
4. records frame / route telemetry;
5. optionally applies the AQM heuristic;
6. decrements SI;
7. rewrites source / destination MAC addresses;
8. rewrites source / destination IPv4 addresses;
9. rewrites both UDP ports to the project service-chain port `6633`;
10. recomputes the IPv4 header checksum and clears the UDP checksum;
11. forwards the original `mbuf` through the selected DPDK port.

### 8.3 Feedback Bypass

Feedback packets are handled before primary frame telemetry and AQM. The implemented reverse forwarding is:

```text
SFF3-facing feedback, SI 255 -> Encoder-facing output, SI 254
Encoder-facing feedback, SI 253 -> SFF1-facing output, SI 252
```

This separation prevents control packets from corrupting primary per-frame counters.

### 8.4 Burst Ownership

A physical SFF2 egress port can carry traffic belonging to different route / frame contexts over time. The implementation therefore associates each pending TX burst with a telemetry owner.

Before traffic from a different owner is placed into the same egress burst, the previous burst is flushed. This avoids attributing packets from one frame or route to another frame's `tx_packets`, `tx_points`, `tx_media_bytes`, or retry counters.

### 8.5 RED-Like AQM: What It Is and What It Is Not

The current SFF2 code contains an experimental congestion heuristic driven by:

```text
virtual_friction in [ 0, 1024 ]
THRESHOLD_LOW  = 512
THRESHOLD_HIGH = 768
```

`virtual_friction` is **not a direct measurement of a hardware or DPDK queue depth**. It is a software proxy updated from TX behaviour:

```text
if nb_tx > 0:
    virtual_friction = max( 0, virtual_friction - nb_tx )

if nb_tx == 0:
    virtual_friction = min( 1024, virtual_friction + BURST_SIZE )
```

With `BURST_SIZE = 32`, repeated zero-accept transmissions cause the score to rise rapidly, while successful TX progress decreases it.

The drop probability for primary, non-EOS traffic is:

```text
P_drop = 0                                      if friction <= 512
P_drop = ( friction - 512 ) / ( 768 - 512 )    if 512 < friction < 768
P_drop = 1                                      if friction >= 768
```

The implementation expresses the middle region as an integer percentage.

Because the mechanism is based on a synthetic score rather than average queue occupancy, it should be described as **RED-like / RED-inspired**, not as a standards-accurate RED implementation.

Feedback traffic is not subjected to this primary-data AQM decision.

### 8.6 Route-Specific Payload Semantics

For Route 0:

```text
logical payload = point bytes
logical frame   = point bytes + one CamHdr
```

For Route 1:

```text
logical payload = MPEG-TS bytes
logical frame   = MPEG-TS bytes + one CamHdr + one EncHdr
```

These definitions intentionally count frame metadata once for logical comparability. They do not represent the repeated per-packet wire overhead.

### 8.7 SFF2 Telemetry

For both implemented primary routes:

```text
tx_duration_ms      = last successful TX - first successful TX
total_residency_ms  = frame_end - first arrival
cycle_ms            = frame_end - previous cycle end
header_wait_ms      = max( 0, cycle - residency )
active_process_ms   = accumulated packet-processing / flush work
node_efficiency_pct = residency / cycle * 100
```

As in `SFF1`, `node_efficiency_pct` is a cycle-occupancy metric and should not be confused with `active_process_ms / residency`.

Logical throughput is:

```text
internal_throughput_mbs = ( logical_RX_frame_bytes / 1,000,000 ) / residency_seconds
```

Logical output bitrate is:

```text
network_bitrate_mbps = logical_TX_frame_bytes * 8 * effective_fps / 1,000,000

effective_fps = TARGET_FPS / temporal_skip
```

For jitter, `SFF2` uses the actual source-frame-ID spacing between received active frames:

```text
frame_delta       = current_frame_id - previous_frame_id
expected_interval = frame_delta / TARGET_FPS
network_jitter_ms = abs( real_interarrival - expected_interval ) * 1000
```

This is important after temporal skipping because consecutive packets seen by `SFF2` need not belong to consecutive source frame IDs.

`camera_to_node_latency_ms` is computed from the Camera timestamp and the first arrival timestamp in the shared host timer domain. `schedule_delay_ms` is calculated against the original source frame IDs, preserving the intended 30-fps timeline even when intermediate frames have been filtered.

Per-frame route counters include:

```text
aqm_drops
tx_retries
virtual_friction snapshot
```

The protocol error fields (`eth_errors`, `ipv4_errors`, `udp_errors`, `nsh_errors`) originate from global cumulative parser counters and are copied into the frame telemetry when a new frame slot is initialised. They should therefore be interpreted as **cumulative snapshots**, not as independent per-frame error deltas.

Route 2 telemetry is intentionally undefined until the Decoder packet format exists.

---

## ⚙️ 9. Encoder — Frame Assembly, Cooperative Polling, CUDA, and H.265

### 9.1 Role

The Encoder is the most computationally complex active node. It combines:

```text
DPDK reception
frame reassembly
project INT metadata consumption
CPU frame-global geometry
CUDA projection
I420 atlas generation
persistent FFmpeg / NVENC encoding
MPEG-TS / PES parsing
DPDK compressed-media transmission
feedback handling
```

The Encoder uses one principal DPDK worker core. Because that same execution context must both process frames and keep receiving network traffic, starvation avoidance is a first-class design constraint.

### 9.2 Frame Assembly

Incoming Route-0 packets are stored in a `FrameBuffer` keyed by `frame_id`.

The network point representation is converted from an array-of-structures form into separate host vectors:

```text
x[]
y[]
z[]
r[]
g[]
b[]
```

This conversion is measured incrementally for each received packet.

On the first packet of a frame, the Encoder stores:

```text
Camera metadata
original_points
Camera TX timestamp
first arrival timestamp
initial payload accounting
```

For every packet, it updates the last-arrival time and the point vectors.

### 9.3 Selecting the Most Complete INT Snapshot

Each SFF1 packet carries a cumulative metadata snapshot. The Encoder compares:

```text
incoming active_point_count
stored   active_point_count
```

and retains the incoming metadata whenever it is at least as complete as the stored state.

The Encoder therefore does not assume that the first or an arbitrary packet contains frame-global reductions.

### 9.4 Frame Readiness and Incomplete Frames

A frame becomes eligible for processing when either:

```text
received_points == original_points
```

or the frame has become sufficiently old and is superseded by a newer frame ( or EOS has been received ).

This prevents a permanently incomplete frame from blocking all subsequent processing. The exported `data_integrity_pct` then makes incomplete input visible rather than silently treating it as a complete frame.

### 9.5 Division of Geometric Work

The Encoder first validates the INT geometry and obtains:

```text
centroid_x = sum_x / active_point_count
centroid_y = sum_y / active_point_count
centroid_z = sum_z / active_point_count
```

It also computes the incoming axis extents:

```text
extent_x = max_x - min_x
extent_y = max_y - min_y
extent_z = max_z - min_z
```

A significant implementation detail is that the current CUDA function explicitly marks these three incoming `extent_*` arguments as unused. The actual projection path recomputes a **transformed GPU bounding box after pose / scaling transformation**. The README therefore does not claim that the SFF1 extents directly determine the current GPU rasterisation scale.

The non-associative frame-global radius remains in the Encoder:

```text
max_r = max_i sqrt( ( x_i - c_x )^2 + ( y_i - c_y )^2 + ( z_i - c_z )^2 )
```

The target radius and final scale are:

```text
target_radius = CAMERA_DIST * 0.2
final_scale   = ( target_radius / max_r ) * zoom
```

with:

```text
CAMERA_DIST = 1200.0
```

### 9.6 Why `CHUNKING_SIZE = 1024` Is Fundamental

The Encoder uses:

```text
CHUNKING_SIZE = 1024 points
```

This value is **not related to the network MTU** and it must not be confused with the `1316 B` MPEG-TS output payload.

It is a **cooperative scheduling granularity** used to protect the DPDK receive path while the Encoder is executing long frame operations on a core-limited machine.

The mechanism appears in three places.

**1. CPU maximum-radius scan**

During the full-frame radius pass:

```text
for each point p:
    if p mod 1024 == 0:
        poll_network_rx()
    evaluate squared radius
```

Instead of monopolising Core 5 for hundreds of thousands of point operations, the Encoder periodically returns to the NIC receive path.

**2. Host-to-device transfers**

The CUDA pipeline copies point arrays to the GPU in 1024-point chunks. Before each chunk is enqueued, the supplied DPDK callback is executed.

This creates repeated opportunities to drain incoming DPDK traffic even while the next frame is being prepared for GPU execution.

**3. CUDA completion waits**

When the host must wait for a CUDA stream, it does not simply block. The code repeatedly performs:

```text
while cudaStreamQuery( stream ) == cudaErrorNotReady:
    poll_network_rx()
```

Thus, GPU execution time becomes useful CPU time for servicing the network.

This cooperative design is one of the principal solutions to the eight-logical-core constraint. It avoids dedicating a second receive core to the Encoder while reducing the risk that long CPU / GPU stages overflow the DPDK receive side.

The trade-off is equally important: a smaller chunk size creates more frequent polling and lower RX starvation risk, but increases callback and loop overhead; a larger chunk size reduces callback overhead but allows longer intervals without servicing the RX path. `1024` is therefore an experimental compromise that must remain fixed when comparing runs.

### 9.7 CUDA Memory Strategy

The CUDA subsystem is designed to minimise cold-start and allocation noise:

```text
MAX_POINTS = 835458
```

Device arrays are allocated before steady-state frame processing. If a frame requires more points than the current allocation, the point buffers are reallocated with doubled capacity.

A small warm-up projection is executed before the main processing path, and the host I420 output buffer is registered with CUDA to support efficient asynchronous device-to-host transfer.

### 9.8 CUDA Projection Stages

The active GPU path performs the following sequence:

```text
1. Chunked Host -> Device copies
2. Centroid-relative translation + yaw / pitch transform + final scaling
3. GPU reduction of transformed x/y/z minima and maxima
4. Transformed bounding-box centre computation
5. global_scale computation
6. G-buffer reset
7. Six-view point projection with atomic depth resolution
8. RGB -> YUV conversion for visible points
9. Geometry / texture / occupancy cross packing
10. I420 device buffer packing
11. Device -> Host copyback
```

The transformed bounding box is used to derive:

```text
bbox_center_axis = ( transformed_min_axis + transformed_max_axis ) / 2

scale_x = transformed_extent_x / WIDTH
scale_y = transformed_extent_y / HEIGHT
scale_z = transformed_extent_z / WIDTH

global_scale = max( scale_x, scale_y, scale_z ) * 1.10
```

The `1.10` factor creates a 10% spatial margin around the transformed content.

### 9.9 Six-View G-Buffer

For every transformed point, the CUDA kernel evaluates six orthogonal views. A depth integer is used with an atomic maximum operation to retain the visible sample for each projected pixel.

For a visible sample, the node writes:

```text
geometry luma
occupancy mask
texture Y
texture U
texture V
```

RGB-to-YUV conversion is performed in the GPU kernel.

### 9.10 Atlas Geometry

Each nominal face is:

```text
WIDTH  = 640
HEIGHT = 480
```

and is stored in a padded cell:

```text
FACE_W_PADDED = 640
FACE_H_PADDED = 512
```

The padded dimensions are multiples of 64, which provides a regular layout compatible with HEVC Coding Tree Unit granularity and prevents nominal face boundaries from being deliberately placed at arbitrary sub-CTU offsets.

The six views are packed into a `4 x 3` cross:

```text
             +---------+
             | Face 4  |
+---------+--+---------+--+---------+---------+
| Face 0  |  | Face 3  |  | Face 1  | Face 2  |
+---------+--+---------+--+---------+---------+
             | Face 5  |
             +---------+
```

The logical cross dimensions are:

```text
CROSS_W = 640 * 4 = 2560
CROSS_H = 512 * 3 = 1536
```

Three crosses are stacked vertically:

```text
Geometry cross
Texture cross
Occupancy cross
```

producing:

```text
ENCODER_W = 2560
ENCODER_H = 4608
```

For I420:

```text
Y plane = 2560 * 4608            = 11,796,480 B
U plane = 1280 * 2304            =  2,949,120 B
V plane = 1280 * 2304            =  2,949,120 B
-----------------------------------------------
Total                              17,694,720 B / frame
```

### 9.11 GPU Timing Probes

The CUDA code exports four event-based intervals:

```text
gpu_transfer_ms
gpu_kernel_ms
gpu_packing_ms
gpu_copyback_ms
```

Their exact interpretation follows CUDA event boundaries:

```text
gpu_transfer_ms = event start -> H2D-complete event

gpu_kernel_ms   = H2D-complete event -> G-buffer-complete event

gpu_packing_ms  = G-buffer-complete event -> I420-pack-complete event

gpu_copyback_ms = I420-pack-complete event -> D2H-complete event
```

`gpu_kernel_ms` should therefore be interpreted as the measured GPU-stream interval for the transformation / bounding-box / G-buffer portion, rather than as the duration of one isolated kernel invocation.

The broader projection measurement is taken in the C++ worker and includes CPU-side geometry and host coordination as well. The residual is reported as:

```text
host_overhead_ms = max( 0, projection_ms - gpu_transfer_ms - gpu_kernel_ms - gpu_packing_ms - gpu_copyback_ms )
```

### 9.12 Persistent FFmpeg / NVENC Process

FFmpeg is created once and kept alive across the complete sequence. Per-frame process creation is deliberately avoided.

The current child process receives:

```text
rawvideo
yuv420p
2560 x 4608
30 fps
```

and encodes with:

```text
codec       = hevc_nvenc
preset      = p2
tune        = ull
rate control= cbr
bitrate     = 10 Mbit/s
maxrate     = 10 Mbit/s
buffer      = 20 Mbit
GOP         = 15
forced IDR  = enabled
container   = MPEG-TS
```

The FFmpeg child is pinned to logical CPU `0`, while the Encoder DPDK worker is assigned logical CPU `5`.

The stdin and stdout pipes are non-blocking during steady-state operation. This allows the C++ worker to continue polling DPDK and draining encoded output instead of blocking indefinitely on the codec process.

### 9.13 Why MPEG-TS / PES Parsing Is Necessary

An FFmpeg stdout pipe is a byte stream. Neither a `read()` return size nor a temporary `EAGAIN` condition represents a video-frame boundary.

The current Encoder therefore reconstructs the output on fixed:

```text
188-byte MPEG-TS packet boundaries
```

and detects the beginning of a video PES by checking:

```text
TS sync byte = 0x47
payload-unit-start-indicator = 1
valid payload offset after any adaptation field
PES prefix = 00 00 01
video stream_id in 0xE0 ... 0xEF
```

At a new video PES:

1. any residual MPEG chunk belonging to the previous frame is emitted;
2. the previous DPDK burst is flushed if required;
3. the oldest pending Encoder timing entry is associated with the new PES;
4. `current_out_frame_id` becomes that source frame ID;
5. `current_mpeg_packet_id` is reset to `0`;
6. `encode_h265_ms` is frozen at the first correctly associated PES output.

A temporary `EAGAIN` may cause a currently accumulated, sub-1316-byte group of complete TS packets to be emitted, but it does **not** change `current_out_frame_id` and is therefore not treated as an encoded-frame boundary.

This distinction prevents compressed bytes from being incorrectly assigned to neighbouring source frames.

### 9.14 Encoder Output Packetisation

Each encoded DPDK packet contains:

```text
NSH-style header
Camera metadata
Encoder metadata
one or more complete MPEG-TS packets
```

`mpeg_bytes_generated` counts only the MPEG-TS media bytes placed into DPDK payloads. It excludes Ethernet / IPv4 / UDP / project metadata overhead.

The DPDK media packet counter is attributed to the corresponding encoded frame through the PES association described above.

### 9.15 Encoder Telemetry — Exact Semantics

**Conversion**

```text
conversion_ms = accumulated time used to copy received point fields from packet AoS layout into host SoA frame vectors
```

It does not represent the entire DPDK receive interval.

**Receive throughput**

```text
receive_span = last_arrival - first_arrival
internal_throughput_mbs = ( logical_payload_bytes / 1,000,000 ) / receive_span
```

The Encoder stores one Camera header in the frame-level logical payload accounting and then accumulates point payload bytes.

**Logical input bitrate**

```text
network_bitrate_mbps = logical_input_bytes * 8 * ( TARGET_FPS / active_skip ) / 1,000,000
```

**Raw-frame queue wait**

```text
wait_raw_queue_ms = projection_start - frame_ready
```

where `frame_ready` is the latest point-conversion completion timestamp recorded while assembling the frame.

**Projection**

```text
projection_ms = projection_end - projection_start
```

This interval includes the CPU frame-global geometry performed after `projection_start` and the complete CUDA projection / copyback path.

**Render / codec-input queue wait**

```text
wait_render_queue_ms = FFmpeg_input_start - projection_end
```

**FFmpeg input handoff**

```text
tx_duration_ms = FFmpeg_stdin_write_end - FFmpeg_stdin_write_start
```

This metric is deliberately different from DPDK-node TX duration. It measures the wall time required to hand the complete `17,694,720 B` I420 frame to the persistent FFmpeg process. It can include non-blocking pipe backpressure and the time spent servicing DPDK / encoded output while waiting for writable pipe capacity.

**H.265 output latency**

```text
encode_h265_ms = first_associated_video_PES_time - FFmpeg_input_start
```

This measurement is asynchronous and is updated when the first correctly associated PES for the frame appears.

Therefore:

```text
encode_h265_ms is NOT added to Encoder residency
encode_h265_ms is NOT a sequential sub-stage after tx_duration_ms in total_processing_ms
```

**Total processing**

```text
total_processing_ms = conversion_ms + projection_ms + tx_duration_ms
active_process_ms    = total_processing_ms
```

**Residency**

```text
total_residency_ms = FFmpeg_stdin_write_end - first_frame_arrival
```

The Encoder's frame exit boundary is the completion of the raw-frame handoff to FFmpeg. Asynchronous compressed output continues afterward and is handled separately.

**Encoder efficiency**

```text
node_efficiency_pct = active_process_ms / total_residency_ms * 100
```

This differs from the SFF duty-cycle definition and must not be silently compared as if every node used the same numerator.

**Camera-relative timing / clock baseline**

The Encoder derives a baseline offset from the first processed frame:

```text
global_clock_offset = first_encoder_arrival - first_camera_TX_timestamp
```

It then reports:

```text
camera_to_node_latency_ms = current_arrival - current_camera_TX - global_clock_offset
```

The result is therefore a **baseline-corrected Camera-relative arrival measurement**, not an independent one-way network-delay estimator.

The partial Encoder-boundary latency is:

```text
end_to_end_latency_ms = Encoder_exit - Camera_TX - global_clock_offset
```

Until Decoder, SFF3, and User exist, this value must be labelled **partial Camera-to-Encoder latency**, not final user-perceived E2E or QoE latency.

**Schedule delay**

```text
real_elapsed  = Encoder_exit - first_encoder_session_arrival
ideal_elapsed = ( frame_id - first_processed_frame_id ) / TARGET_FPS
schedule_delay_ms = ( real_elapsed - ideal_elapsed ) * 1000
```

Using source frame IDs preserves the original temporal timeline even when `SFF1` has removed intermediate frames.

**Jitter**

The current Encoder compares observed interarrival against the active skip:

```text
network_jitter_ms = abs( real_interarrival - active_skip / TARGET_FPS ) * 1000
```

**Data integrity**

```text
data_integrity_pct = rx_points / original_points * 100
```

**Important `tx_points` interpretation:** the Encoder sets `tx_points` equal to the number of points represented by the encoded frame. It does **not** mean that raw point records are transmitted on Route 1; Route 1 carries MPEG-TS media.

**Compressed output indicators**

```text
mpeg_bytes_generated = MPEG-TS bytes attributed to the frame
tx_packets           = DPDK compressed-media packets accepted for that frame
tx_retries           = zero-accept DPDK TX-burst events
mbuf_starvation      = output mbuf allocation / append failures tracked by the Encoder media path
```

### 9.16 Encoder End-of-Stream Handling

After input EOS and after all buffered frames have been processed, the Encoder:

1. closes FFmpeg stdin;
2. removes non-blocking mode from FFmpeg stdout;
3. drains encoded output until true EOF;
4. processes the residual stream through the same MPEG-TS / PES parser;
5. emits any residual current MPEG chunk;
6. flushes the final DPDK burst;
7. waits for the FFmpeg child;
8. emits an Encoder-side EOS packet toward `SFF2`;
9. exports telemetry.

This guarantees that normal asynchronous codec output is drained before the chain is terminated.

---

## 🖥️ 10. CPU, GPU, and Core-Constrained Execution

The experimental host exposes eight logical CPUs ( `0-7` ). Core placement is therefore part of the system architecture and must be treated as an experimental variable.

| Logical CPU | Assignment | Current Rationale |
|---:|---|---|
| `0` | Host / OVS auxiliary work + Encoder FFmpeg child + future Decoder FFmpeg child | Housekeeping / auxiliary-core compromise on an eight-core system |
| `1` | OVS-DPDK PMD | Dedicated virtual-switch packet-processing core |
| `2` | Camera | Dedicated DPDK source worker |
| `3` | SFF1; future SFF3 shares the container cpuset | Only SFF1 is active in the currently validated upstream hot path |
| `4` | SFF2 | Dedicated four-port DPDK forwarder |
| `5` | Encoder | Principal Encoder DPDK / C++ worker |
| `6` | Future Decoder | Reserved for Decoder processing |
| `7` | Future User | Reserved for rendering / user logic |

The current OVS configuration uses:

```text
dpdk-lcore-mask = 0x1   -> Core 0
pmd-cpu-mask    = 0x2   -> Core 1
dpdk-socket-mem = 1024 MiB
```

The host setup allocates:

```text
4096 x 2 MiB HugePages = 8 GiB
```

### Core 0 Sharing

Running FFmpeg on Core 0 is a deliberate compromise, not a cost-free decision. It prevents the codec child from consuming one of the dedicated DPDK worker cores, but it may contend with host housekeeping and the OVS auxiliary lcore.

For the current upstream experiment this arrangement is acceptable as long as it is held constant and reported. Once Decoder becomes active and launches its own codec process, Core 0 contention must be re-evaluated rather than assumed to remain negligible.

### Why the Current Constraint Is Manageable

The present implementation reduces CPU pressure through:

```text
dedicated OVS PMD placement
DPDK worker pinning
optional Linux CPU isolation
SFF1 associative reduction offload
GPU projection
NVENC compression
persistent FFmpeg processes
Encoder cooperative polling during CPU and GPU work
```

The final complete chain will contain more simultaneous work than the current upstream subset, so the resource allocation remains an experimental design decision rather than a permanently solved scaling problem.

---

## 11. HugePages and Optional CPU Isolation

DPDK memory is backed by HugePages configured by the project launcher.

The optional isolation scripts configure:

```text
isolcpus=1-7
```

leaving Core `0` as the primary non-isolated housekeeping CPU.

Typical usage from `src/` is:

```bash
sudo ./enable_isolcpus.sh
sudo reboot
```

After reboot:

```bash
cat /proc/cmdline
```

and verify that `isolcpus=1-7` is present.

To restore the default scheduler configuration:

```bash
sudo ./disable_isolcpus.sh
sudo reboot
```

CPU isolation changes the experimental environment. Benchmark results produced with different isolation, Docker cpuset, OVS PMD, or FFmpeg-affinity settings must be treated as different configurations.

---

## 12. Container and OVS-DPDK Environment

The common container image is based on:

```text
nvidia/cuda:12.2.0-devel-ubuntu22.04
```

and builds:

```text
DPDK 22.11.4
```

from source. The image also includes GCC / G++, Meson, Ninja, NUMA development support, FFmpeg, `tcpdump`, `ethtool`, and related utilities.

Containers use:

```text
--net none
--privileged
```

and communicate through explicitly mounted vhost-user sockets rather than Docker's ordinary network stack.

The service functions receive mounts for:

```text
/dev/hugepages
/tmp
/shared
/app
```

The project source directory is mounted as `/app`, so each node entrypoint compiles or launches the current repository code rather than a stale copy embedded in the image.

### OVS-DPDK Topology

The virtual switch uses a `netdev` bridge named:

```text
br-sfc
```

The physical service adjacency is deterministic and can be represented horizontally as:

```text
Camera <-> SFF1 <-> SFF2 <-> Encoder
                     |  |
                     |  +-> Decoder
                     +----> SFF3 <-> User
```

The bridge only forwards explicitly defined adjacent links; unmatched traffic is dropped. The service-path logic remains inside the DPDK functions rather than inside OVS.

> Debug mirroring and packet-level tracing can perturb timing. They should be enabled for functional inspection and disabled for final benchmark runs unless instrumentation overhead is itself part of the experiment.

---

## 13. Repository Structure

The current repository organisation is conceptually:

```text
Thesis/
├── README.md
├── docs/                                  # Thesis / reference material
├── env/                                   # Python virtual environment
└── src/
    ├── infrastructure/
    │   ├── setup_topology.sh
    │   └── start_microservices.sh
    │
    ├── microservices/
    │   ├── base/
    │   │   └── Dockerfile
    │   ├── camera/
    │   │   ├── c/camera.c
    │   │   └── entrypoint.sh
    │   ├── sff1/
    │   │   ├── c/sff1.c
    │   │   └── entrypoint.sh
    │   ├── sff2/
    │   │   ├── c/sff2.c
    │   │   └── entrypoint.sh
    │   ├── encoder/
    │   │   ├── cpp/encoder.cpp
    │   │   ├── cu/encoder.cu
    │   │   ├── h/encoder.h
    │   │   ├── Makefile
    │   │   └── entrypoint.sh
    │   ├── decoder/                       # Under development
    │   ├── sff3/                          # Under development
    │   └── user/                          # Under development
    │
    ├── shared/
    │   ├── data/loot/
    │   │   ├── original/                  # Original PLY sequence
    │   │   ├── bin/                       # Header-less 16-B/point frames
    │   │   └── produced/                  # Reserved generated outputs
    │   ├── log/
    │   │   ├── converter/
    │   │   ├── camera/
    │   │   ├── sff1/
    │   │   ├── sff2/
    │   │   └── encoder/
    │   └── py/
    │       └── converter/
    │           └── converter.py
    │
    ├── enable_isolcpus.sh
    ├── disable_isolcpus.sh
    ├── init_all.sh
    └── stop_all.sh
```

The intentionally removed `shared/py/pipeline/` and `shared/log/pipeline/` entries are not listed as active repository components.

---

## 🐍 14. Python Environment and Dataset Preparation

The root-level:

```text
env/
```

is the Python environment used by the current offline utilities, primarily the point-cloud converter.

Activate it from the repository root with:

```bash
source env/bin/activate
```

For the current converter, the essential third-party packages are:

```text
numpy
plyfile
```

If the environment must be recreated:

```bash
python3 -m venv env
source env/bin/activate
python -m pip install --upgrade pip
python -m pip install numpy plyfile
```

`pandas` and `matplotlib` are not required merely to run the present converter; they become relevant again only if higher-level analysis tooling is restored.

### Offline Converter

The converter transforms the original PLY sequence into contiguous, header-less point records matching `point_tx`:

```text
float32 x
float32 y
float32 z
uint8   r
uint8   g
uint8   b
uint8   padding
```

The current scale factor is:

```text
SCALE_FACTOR = 1.0
```

The resulting binary frame can be consumed directly by the Camera without parsing a PLY header during the streaming hot path.

Typical execution is:

```bash
source env/bin/activate
python3 src/shared/py/converter/converter.py
deactivate
```

The converter is an **offline preparation stage**; its runtime should not be merged with the Camera's DPDK streaming metrics.

---

## 15. 🚀 Running the Experiment

### 15.1 Prepare the Dataset

From the repository root:

```bash
source env/bin/activate
python3 src/shared/py/converter/converter.py
deactivate
```

### 15.2 Optional: Enable CPU Isolation

From `src/`:

```bash
sudo ./enable_isolcpus.sh
sudo reboot
```

After reboot, return to the repository and verify `/proc/cmdline` before benchmarking.

### 15.3 Start the Environment

From `src/`:

```bash
sudo ./init_all.sh
```

The launcher is responsible for the host-side DPDK preparation and for invoking the topology / container startup sequence. When `init_all.sh` is used, the infrastructure scripts should not be launched a second time unless the user intentionally tears down and reconstructs the topology.

### 15.4 Inspect the Active Nodes

```bash
sudo docker ps
sudo docker logs camera
sudo docker logs sff1
sudo docker logs sff2
sudo docker logs encoder
```

### 15.5 Stop the Experiment

```bash
sudo ./stop_all.sh
```

CPU isolation is a reboot-level setting and remains independent of the ordinary container / OVS shutdown path.

---

## 16. Entrypoint Execution Model

Each implemented native node is launched by an `entrypoint.sh` inside its container.

The entrypoint typically:

1. compiles the current mounted source with optimisation enabled;
2. links against the DPDK libraries available in the common image;
3. launches the EAL on the logical core passed through `DPDK_CORE`;
4. creates one or more `virtio_user` devices;
5. binds them to the corresponding `/tmp/vh-*` vhost-user sockets;
6. starts the node's run-to-completion loop.

For example, the Camera is compiled with an optimised GCC invocation linked through `pkg-config` and is launched directly against its virtio-user vhost socket.

This runtime compilation approach is useful during thesis development because the mounted source remains the single authoritative implementation.

---

## 📊 17. Native Telemetry Files

The current snapshot exports node telemetry under:

```text
src/shared/log/
```

Main files include:

| Component | Telemetry |
|---|---|
| Converter | `log/converter/telemetry_converter.csv` |
| Camera | `log/camera/telemetry_camera.csv` |
| SFF1 | `log/sff1/telemetry_sff1.csv` |
| SFF2 Route 0 | `log/sff2/telemetry_sff1_enc.csv` |
| SFF2 Route 1 | `log/sff2/telemetry_enc_dec.csv` |
| SFF2 Route 2 | `log/sff2/telemetry_dec_sff3.csv` ( reserved; meaningful frame semantics not yet defined ) |
| Encoder | `log/encoder/telemetry_encoder.csv` |
| FFmpeg | `log/encoder/ffmpeg.txt` |

There is presently **no active cross-node pipeline analysis script** documented by this README. The raw per-node CSV files are the authoritative measurement outputs for the current repository snapshot.

### Timing Quantities Must Not Be Added Indiscriminately

The telemetry intentionally contains metrics from different semantic domains:

```text
Camera pacing-excluded TX wall-time
DPDK API active TX time
SFF frame residency
SFF inter-frame waiting
Encoder receive conversion
CPU / CUDA projection
FFmpeg input handoff
asynchronous first-PES encode latency
```

For example:

```text
Encoder total_processing_ms = conversion_ms + projection_ms + tx_duration_ms
```

but:

```text
encode_h265_ms
```

is asynchronous and must **not** be added to that sum.

Likewise, for `SFF1` and `SFF2`:

```text
cycle_ms = header_wait_ms + total_residency_ms
```

whereas `active_process_ms` is a separate sub-measurement of work and is not the quantity used in their current efficiency formula.


## 🧪 18. Representative Results from Validated Experimental Runs

The repository is still under active development, and the complete `Camera -> User` chain is not yet available. Nevertheless, the upstream implementation has already undergone repeated end-to-end validation through the currently active nodes.

> **Interpretation rule:** the values below consolidate representative runs produced during the present validation cycle. They are not all necessarily taken from one single execution, because specific runs were used to validate different subsystems ( source pacing, relay semantics, Encoder timing, and MPEG-TS / PES attribution ). They should therefore be read as **validated experimental evidence**, not as a final synchronized benchmark of the future complete chain.

### 18.1 Upstream Point-Cloud Transport

A representative `CACHE_MODE_MIDDLE` run produced point-cloud frames containing approximately:

```text
Mean transmitted points        ~= 793,821 points / frame
Mean raw point payload         ~= 12.7 MB / frame
Logical Camera bitrate         ~= 3.05 Gbit/s at the nominal 30-fps definition
Camera internal throughput     ~= 1.33 GB/s
```

These values demonstrate the load imposed by an uncompressed volumetric source before geometric video coding. The logical bitrate is intentionally derived from the frame payload and the configured target frame rate; it is **not** a physical Ethernet wire-rate measurement.

The same storage-aware run showed approximately:

```text
Camera disk I/O                ~= 96.4 ms / frame
Camera TX wall-time            ~= 9.60 ms / frame   ( deliberate pacing excluded )
Camera active rte_eth_tx time  ~= 1.85 ms / frame
Camera total residency         ~= 106.0 ms / frame
mbuf starvation                = 0
```

The approximately `96 ms` file-read cost is larger than the nominal `33.33 ms` frame period. Consequently, `CACHE_MODE_MIDDLE` becomes primarily **source-I/O limited**, and the effective cadence falls to roughly `9.4 frames/s` instead of the nominal 30 fps.

This is an important experimental result rather than a telemetry defect: in `CACHE_MODE_MIDDLE`, storage access is deliberately retained in the frame path. The condition must therefore remain separate from `CACHE_MODE_BEST`, in which the dataset is already resident in the application buffers before the timed streaming loop.

### 18.2 Pacing and Backpressure Observation

In the same storage-aware condition, the Camera observed a high number of zero-accept TX events, on the order of:

```text
tx_retries ~= 10.2 x 10^3 events / frame
```

while `mbuf_starvation` remained zero.

This result explains why pacing is an architectural requirement rather than an aesthetic optimisation. Once disk I/O has already consumed most or all of the nominal frame budget, the Camera may have little positive pacing window left and must deliver a large point-cloud frame through a finite virtio-user / OVS / receiver path over a short interval.

The counter must be interpreted correctly:

```text
tx_retries != packet loss
tx_retries != UDP retransmissions

tx_retries = number of rte_eth_tx_burst() calls that accepted zero pending packets
```

The observation therefore captures **local DPDK backpressure**. It also motivates keeping `PACING_MODE`, cache mode, ring configuration, OVS PMD placement, and core affinity fixed when comparing experimental runs.

### 18.3 SFF1 In-Network Processing

The validated SFF1 run showed approximately:

| Metric | Representative value |
|---|---:|
| `active_process_ms` | `2.26 ms` |
| `tx_duration_ms` | `15.60 ms` |
| `total_residency_ms` | `15.61 ms` |
| `header_wait_ms` | `90.00 ms` |
| `cycle_ms` | `105.61 ms` |
| `node_efficiency_pct` | `15.82 %` |
| `camera_to_node_latency_ms` | `0.255 ms` |
| `tx_retries` | `0` |

The most significant observation is that the service function performs the cumulative sums, extrema, counters, temporal filtering, header replacement, and forwarding without introducing TX retry pressure in the validated run.

The low `active_process_ms` relative to the Camera's storage cost is important for the thesis objective: the associative geometric reductions have been moved into the forwarding path while retaining a comparatively small measured CPU-processing interval.

`header_wait_ms` must not be interpreted as processing overhead. It represents the idle portion of the inter-frame cycle and is intentionally separated from the approximately `15.6 ms` frame residency.

### 18.4 SFF2 Route 0 Forwarding

For the validated `SFF1 -> SFF2 -> Encoder` route:

| Metric | Representative value |
|---|---:|
| `active_process_ms` | `0.905 ms` |
| `tx_duration_ms` | `15.56 ms` |
| `total_residency_ms` | `15.56 ms` |
| `header_wait_ms` | `90.05 ms` |
| `cycle_ms` | `105.61 ms` |
| `node_efficiency_pct` | `15.76 %` |
| `camera_to_node_latency_ms` | `0.393 ms` |
| TX retries / parser errors | `0` in the validated run |

This result supports the architectural separation between **service computation** and **service steering**. Route 0 adds the required service-path forwarding logic while its measured active processing remains below one millisecond on average in the representative run.

Point-count checks across:

```text
Camera TX -> SFF1 RX -> SFF1 TX -> SFF2 Route 0 RX/TX -> Encoder RX
```

were validated without point mismatches for the analyzed frames.

### 18.5 Encoder Input Integrity and Data-Plane Stability

Across the validated Encoder runs:

```text
data_integrity_pct = 100 %
tx_retries         = 0
mbuf_starvation    = 0
```

for the successfully processed frame population.

The geometric conversion stage remained on the order of a few milliseconds per frame, while the CUDA projection path was typically on the order of tens of milliseconds. These measurements are deliberately kept distinct from the asynchronous H.265 output timing.

The result is particularly relevant to the cooperative-polling strategy described earlier: despite the full-frame `max_r` scan, CUDA transfers, GPU execution, I420 generation, and non-blocking FFmpeg communication, the Encoder was able to continue servicing DPDK without observed output `mbuf` starvation or TX retry events in the validated 300-frame runs.

### 18.6 Validation of MPEG-TS / PES Frame Attribution

The previous EAGAIN-based attribution heuristic was replaced by MPEG-TS / PES-aware parsing because a pipe read boundary is not a video-frame boundary.

The revised implementation was validated on a complete 300-frame run and detected:

```text
video PES -> source frame 1
video PES -> source frame 2
...
video PES -> source frame 300
```

with exactly one new video-PES transition for each source frame and no missing or duplicated frame identifier in the observed sequence.

A concrete example is the final frame:

```text
Encoder frame_id                   = 300
mpeg_bytes_generated               = 28,764 B
28,764 / 188                       = 153 complete MPEG-TS packets
ceil( 153 / 7 )                    = 22 DPDK media chunks
Encoder tx_packets                 = 22
FFmpeg coded-frame size            = 28,057 B
```

The equality between the mathematically expected `22` groups and the exported `tx_packets = 22` directly validates the current `7 x 188 B` packetisation logic for that frame.

The difference between the FFmpeg coded-frame size and the MPEG-TS bytes is expected because MPEG-TS / PES multiplexing introduces container overhead.

### 18.7 Codec-Level Observation

In a representative 300-frame FFmpeg / NVENC validation run:

```text
I frames                = 20
P frames                = 280
B frames observed       = 0
Average coded bitrate   ~= 8.02 Mbit/s
```

with `GOP = 15`, consistent with the configured periodic intra-refresh structure used by the current command line.

The aggregate sizes measured in that validation run were:

```text
FFmpeg coded frame bytes    =  9,979,541 B
Encoder MPEG-TS bytes       = 10,272,884 B
MPEG-TS / mux overhead      =    293,343 B
Relative overhead           ~= 2.94 %
```

This is a useful integrity check because the earlier frame-attribution issue affected **which source frame received a given output chunk**, but it did not invalidate the aggregate compressed-stream byte accounting.

### 18.8 What These Results Establish

At the current implementation stage, the validated runs support the following conclusions:

- the raw point-cloud path preserves point counts through the implemented upstream service functions;
- `SFF1` successfully performs the selected associative geometric reductions in the data plane;
- `SFF2` Route 0 performs service steering with low measured active-processing cost in the representative run;
- the Encoder receives complete point sets in the validated population and remains free of observed TX retries / `mbuf` starvation;
- the cooperative polling strategy is sufficient for the currently active Encoder workload on the available core budget;
- the MPEG-TS parser now attributes compressed traffic to source frames using PES semantics rather than pipe-drain timing;
- the `7 x 188 B` MPEG-TS packetisation is both MTU-safe and experimentally consistent with exported per-frame packet counts;
- the principal bottleneck in `CACHE_MODE_MIDDLE` is the deliberately included Camera-side storage access, not evidence of a point-loss failure in the implemented data-plane chain.

These conclusions apply to the **currently validated upstream system only**. They do not yet establish final Camera-to-User latency, decode performance, reconstructed point-cloud quality, interaction latency, or full-chain QoE.

---

## 19. ⚠️ Experimental Interpretation and Known Boundaries

### 19.1 Logical Bytes vs Wire Bytes

The throughput / bitrate metrics intentionally use logical frame bytes to preserve comparability with application-level processing boundaries.

They exclude repeated per-packet Ethernet / IPv4 / UDP / service metadata overhead. They must therefore not be presented as physical NIC or vhost-user wire rates.

### 19.2 Cache Mode Is Part of the Experiment

`CACHE_MODE_MIDDLE` includes frame file reads in Camera residency. A `CACHE_MODE_MIDDLE` result must not be compared with a `CACHE_MODE_BEST` result as if the Camera workload were identical.

### 19.3 Pacing Is Part of the Experiment

Pacing changes the temporal distribution of packet bursts and directly affects local TX-ring pressure and retry behaviour. Benchmark runs with different pacing settings are different source-load experiments.

### 19.4 Core Affinity Is Part of the Experiment

Changing any of the following changes the execution environment:

```text
isolcpus
Docker cpusets
OVS lcore placement
OVS PMD placement
FFmpeg affinity
```

### 19.5 Route 2 Is Not Implemented

Although port, SPI / SI, telemetry array, and output path placeholders exist, the Decoder-side Route-2 packet parser is explicitly undefined in the present `SFF2` source.

### 19.6 Final E2E / QoE Is Not Yet Available

The current Encoder metric named `end_to_end_latency_ms` reaches only the Encoder's FFmpeg-input handoff boundary after baseline correction.

It is therefore a **partial Camera-to-Encoder measurement**. Final Camera-to-User latency, decode latency, reconstruction quality, interaction latency, command-to-photon behaviour, and user-perceived QoE require Decoder, SFF3, and User.

### 19.7 NSH Interoperability Is Not Claimed

SPI / SI service semantics are inspired by NSH and the SFC architecture, but the custom fixed INT representation is not RFC 8300 MD-Type-2 TLV encoding. This distinction should remain explicit in the thesis and README.

### 19.8 AQM Is Experimental

`virtual_friction` is a synthetic congestion state, not an observed queue depth. The mechanism is useful as an in-data-plane experimental control but should not be evaluated as an implementation of canonical RED without additional queue-state modelling.

---

## 🛠️ 20. Main Engineering Challenges and Current Solutions

### Limited Logical Cores

**Challenge:** the target chain contains more active roles than can be given completely independent CPU resources on the current eight-logical-core machine.

**Current approach:** dedicated DPDK cores for active native nodes, a dedicated OVS PMD, GPU / NVENC offload, controlled core sharing for inactive / auxiliary roles, and cooperative polling in the Encoder.

### DPDK Backpressure at the Source

**Challenge:** a point-cloud frame contains enough packets to overrun local producer-consumer queues when submitted as a microburst.

**Current approach:** burst pacing, zero-accept retry accounting, bounded retry loops, and per-frame retry telemetry.

### Performing Useful Work In-Network

**Challenge:** not every geometric operation is suitable for incremental packet processing.

**Current approach:** move only sums, extrema, counters, and temporal filtering to `SFF1`; retain centroid-dependent radius, visibility, projection, and compression in the Encoder.

### Protecting RX While Computing

**Challenge:** a single Encoder worker can otherwise spend long intervals away from DPDK reception.

**Current approach:** 1024-point cooperative polling during the CPU radius scan and H2D submission, plus polling while CUDA streams are incomplete and while FFmpeg stdin is backpressured.

### Preserving MTU Across Different Traffic Types

**Challenge:** raw point packets and compressed media carry different metadata stacks.

**Current approach:** derive payload constants from the final envelope: `80 x 16 B` points upstream and `7 x 188 B` TS packets downstream.

### Correctly Attributing Asynchronous Codec Output

**Challenge:** FFmpeg pipe availability is not a video-frame delimiter.

**Current approach:** reconstruct fixed TS packets, detect video PES starts, associate those starts with a FIFO of Encoder input timestamps, and restart media `packet_id` for each encoded frame.

---

## ✅ 21. Reproducibility Checklist

Every archived benchmark should preserve at least:

```text
source revision
host kernel
CPU model and logical CPU count
GPU model
NVIDIA driver
CUDA version
DPDK version
Open vSwitch version
Docker version
FFmpeg version

HugePage configuration
OVS lcore placement
OVS PMD placement
Docker cpusets
isolcpus state
FFmpeg CPU affinity

Camera cache mode
Camera pacing mode
PACING_MARGIN
TARGET_FPS
frame count
POINTS_PER_PACKET
BURST_SIZE
MAX_RETRIES

SFF1 temporal skip policy
SFF1 project INT layout
SFF2 AQM thresholds
SFF2 virtual-friction policy

Encoder CHUNKING_SIZE
MAX_POINTS
face / atlas dimensions
CUDA warm-up state
target H.265 bitrate
NVENC preset / tune
GOP length
MPEG-TS payload size

debug / mirror state
all native telemetry CSV files
FFmpeg statistics / stderr logs
```

A benchmark is reproducible only when its results and the configuration that produced them are stored together.

---

## 🚧 22. Ongoing Work

The principal remaining implementation stages are:

```text
1. Decoder
2. Decoder packet-format definition
3. SFF2 Route 2 implementation and telemetry validation
4. SFF3
5. User / renderer
6. complete feedback-loop validation
7. final Camera-to-User E2E latency
8. decode / reconstruction timing
9. visual and geometric quality evaluation
10. final full-chain core-allocation re-evaluation
```

The previously removed cross-node Python analysis layer can be reintroduced after the current native telemetry interfaces are considered stable.

---

## 23. 📚 References

1. J. Halpern and C. Pignataro, **Service Function Chaining ( SFC ) Architecture**, RFC 7665, IETF, 2015.
2. P. Quinn, U. Elzur, and C. Pignataro, **Network Service Header ( NSH )**, RFC 8300, IETF, 2018. The present project uses its SPI / SI terminology and architectural concepts but does not claim full MD-Type-2 wire-format compliance.
3. **DPDK Project**, Data Plane Development Kit documentation; project container currently builds DPDK 22.11.4.
4. **Open vSwitch Project**, Open vSwitch and OVS-DPDK documentation.
5. **NVIDIA**, CUDA Toolkit documentation and NVIDIA Video Codec / NVENC documentation.
6. **FFmpeg Project**, FFmpeg documentation.
7. Maria Giovanna Lacaria, **Point Cloud Coding for Extended Reality Services**, Master's Thesis, Sapienza University of Rome, Academic Year 2025/2026.

---

## Final Note

The principal contribution of this repository is the **co-design of packet transport and computation**.

The point cloud is not merely carried between isolated applications. Instead, selected reductions are executed while packets traverse the DPDK service chain, the forwarding layer preserves frame-aware metadata, and the Encoder is explicitly designed to continue servicing DPDK while CPU, GPU, and codec stages are active.

The resulting platform is therefore best interpreted as an experimental study of **where volumetric-streaming computation can be placed, which operations can be safely moved into the data plane, and which costs or scheduling mechanisms emerge when that decision is implemented on a core-constrained DPDK system**.
