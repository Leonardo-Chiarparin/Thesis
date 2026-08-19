# 🌐 "DPDK"-based "Service Function Chaining" for Real-Time Point-Cloud Streaming
> **Experimental Thesis Project — Sapienza University of Rome**<br>

> A data-plane-oriented architecture for real-time-oriented volumetric point-cloud transport, in-place geometric aggregation, "GPU" projection, & hardware-accelerated "H.265" delivery.

### 👥 Academic Information
**Author:** Leonardo Chiarparin ( Student ID: **2016363** )<br>

**Thesis Supervisor:** Professor Marco Polverini<br>

**Degree Programme:** Engineering in Computer Science<br>

**Institution:** **Sapienza University of Rome**

---

## 🧭 Realization Status

This repository is an experimental research platform rather than a production-ready "Service Function Chaining" ( "SFC" ) framework. The current snapshot implements & evaluates the upstream portion of the intended volumetric chain, while the downstream reconstruction & user-facing stages are still under development.

| Node | Condition | Responsibility |
|---|---|---|
| `Camera` | Validated | "DPDK"-native point-cloud source, warm-mode file acquisition, frame packetisation, absolute scheduling, temporal selection, & initial telemetry |
| `SFF1` | Validated | "Geometry-Aware Classifier" ( "GAC" ) implementing packet-level spatial aggregation, exact frame-completing radius evaluation, experimental "Network Service Header" ( "NSH" ) metadata insertion, & control ( "Temporal" ) decapsulation to `Camera` |
| `SFF2`<br>( Route 0 ) | Validated | Stateful dispatcher & proxy for the `SFF1` -> `SFF2` -> `Encoder` that removes outer encapsulation before "SFC"-unaware application elements, preserving relevant application content |
| `Encoder` | Validated | "SFC"-unaware frame assembly, geometry-offload consumption or local fallback retrieval, workload-driven monitoring, "CUDA" projection, "FFmpeg" / "NVENC" pre-roll & encoding, "MPEG-TS" chunking & attribution, & "UDP" output segmentation |
| `SFF2`<br>( Route 1 ) | Validated | Proxy-maintained `Encoder` -> `SFF2` -> `Decoder` path ( "Main" ) transition & compressed-media relay heading for prospective `Decoder` |
| `Decoder` | Under<br>development | "H.265" decoding & spatial reconstruction, intended to remain "SFC"-unaware & receive an ordinary application datagram from the proxy boundary |
| `SFF2`<br>( Route 2 ) | Reserved, not operational | Upcoming `Decoder` -> `SFF2` -> `SFF3` passage. The application format & route-specific telemetry are intentionally left undefined until the `Decoder` output contract is stabilised |
| `SFF3` | Under<br>development | Final data path stage & user-side command entrypoint ( "Pose" ) |
| `User` | Under<br>development | Rendering, interaction, zoom generation, & client-perceived "Quality of Experience" ( "QoE" ) measurements |

The **presently sustained upstream chain** is represented horizontally as:

```
Camera -> SFF1 -> SFF2 ( Route 0 ) -> Encoder -> SFF2 ( Route 1 )
```

The **complete target primary route**, namely **"Main"**, stays:

```
Camera -> SFF1 -> SFF2 ( Route 0 ) -> Encoder -> SFF2 ( Route 1 ) -> Decoder -> SFF2 ( Route 2 ) -> SFF3 -> User
```

Control is deliberately separated into two independent logical service paths rather than being represented as one monolithic feedback packet:

```
"Temporal" : Encoder -> SFF2 -> SFF1 -> Camera
"Pose"     : User -> SFF3 -> SFF2 -> Decoder
```

The temporal loop is fully operational. `Encoder` determines a `temporal_skip`, `SFF2` classifies the plain datagram into the corresponding chain, `SFF1` eliminates the envelope, & `Camera` applies the requested factor before subsequent source-frame transmission. The representative validation run remains at `current_skip = 1`, because `Encoder` workload never satisfies the configured indicators.

"Pose" evolution is not yet active. Consequently, `yaw = 0`, `pitch = 0`, & `zoom = 1` remain static references for the complete validated upstream course. However, the planned design is to originate updated information at the `User`, move it through `SFF3` & `SFF2`, & expose it to `Decoder` without requiring any node to become responsible for client-driven quality adjustment.

> **Repository note:** this README documents the native telemetry exported by `Camera`, `SFF1`, `SFF2`, & `Encoder`, together with the offline Converter & the independent "FFmpeg" statistics used for cross-checking.

> **Validation scope:** the quantitative results reported below refer to the representative 300-frame "Loot" experiment with the precise compile-time & launcher setting present in this snapshot. In particular, `Camera` exploits `CACHE_MODE_MIDDLE` together with `WARM_MODE_ENABLED`. `Encoder` uses `OFFLOAD_MODE_ENABLED` & `TEMPORAL_ADAPTATION_ENABLED`. The measured `Camera` start-to-start interval is approximately `33.330 ms`, corresponding to approximately `30.003 frames/s`. This result establishes the nominal upstream source cadence for the measured configuration, but, for intellectual honesty, it is not yet a complete `Camera`-to-`User` real-time proof since `Decoder`, `SFF3`, & `User` are still missing.

---

## 🎯 1. Project Motivation & Research Objective

The project investigates whether selected functions of a real-time volumetric streaming pipeline can be moved away from conventional application-level microservices & executed directly while packets traverse a "DPDK"-based "SFC".

The objective is **not** merely to replace kernel sockets with a faster packet-I / O "API". The central research question is whether the forwarding path can become an active computational component without overloading the data plane or compromising the semantic correctness of frame-level processing, thereby preserving bounded queuing conduct, integrity, & sufficient observability to attribute latency to components that actually produce it.

### 1.1 Why "DPDK" & "UDP" Are Topological Requirements

"DPDK" is employed because the experiment needs more than aggregate throughput. Service functions must be able to inspect & modify the packet envelope directly, retain per-frame state, perform incremental computation while traffic is in flight, observe local queue acceptance, & account separately for active `rte_eth_tx_burst()` execution & wall-clock backpressure. A conventional kernel-socket path would deliberately hide part of that packet lifecycle behind socket queues, scheduler wake-ups, generic buffering, & transport-stack policy. Those mechanisms are valuable for general-purpose applications, but they would make the principal experimental boundary less explicit.

"UDP" is equally required. Volumetric frames already carry their own identity, packet sequence, original-point count, & temporal relevance. The application therefore benefits from preserving datagram boundaries & deciding explicitly which information remains useful. A reliable ordered byte stream such as "TCP" would couple later data to retransmission & in-order delivery of earlier missing bytes, introducing head-of-line blocking precisely when a late volumetric frame can be less valuable than a newer one. It would also move flow control & recovery policy into the transport layer, while this project intentionally studies source-rate regulation through an application-aware temporal controller.

This choice does **not** assume that "UDP" is trustworthy. Instead, correctness is made observable through explicit frame & packet identifiers, point / media counters, integrity percentages, validation of protocol fields, redundant "End-of-Stream" ( "EOS" ) signalling, & bounded local resubmission when a Tx ring temporarily accepts zero elements.

**Accordingly**, the architecture employs "DPDK" & "UDP" to obtain:

```
explicit packet ownership & lifetime
application-preserved datagram boundaries
in-place service-header manipulation
observable local queue backpressure
frame-aware loss / integrity accounting
"Maximum Transmission Unit" ( "MTU" )-bounded packetisation
service-path-specific metadata processing
control over temporal relevance at the source
```

Performance is a consequence of the design, but **experimental control, packet semantic visibility, & avoidance of transport-level head-of-line coupling are equally important motivations**.

### 1.2 Selecting Computation for the Data Plane

The design separates operations according to their mathematical dependency structure.

Operations that are **associative, incrementally composable, & element-progressive** are natural candidates for in-network execution ( "GAC" ):

```
sum_x, sum_y, sum_z
min_x, min_y, min_z
max_x, max_y, max_z
active_point_count
```

From these running quantities, `SFF1` can expose significant values such as:

```
C_N = ( 1 / N ) * sum_i( p_i )
E_N = p_max,N - p_min,N
B_N = ( p_min,N + p_max,N ) / 2
```

where `C_N` is the centroid of the points observed so far, while `E_N` & `B_N` are respectively the current axis-aligned extent & bounding-box centre.

Clearly, not every geometric entity can be made exact before frame completion, such as the maximum outer radius:

```
max_r = max_i || p_i - C_final ||_2
```

It depends on `C_final`, which is unknown until the frame has been received entirely. "GAC" therefore keeps a compact frame-local `XYZ` workspace, performs the first aggregation pass while packets are already being forwarded, & executes the precise distance pass only when the final population is known. The result is attached to the scene-completing packet.

Such limitation is not a missing optimisation that can be solved by simply sending a different packet earlier. Pre-computing the accurate centroid-dependent boundary at the `Camera`, in an offline stage, or in another processing element would move the computation outside the data-plane location that the experiment is intended to evaluate. Likewise, pose-dependent transformed frontiers cannot be finalised before the relevant stance is available. Consequently, the implementation distinguishes rigorously between **continuous in-path information** & **frame-global information that is mathematically unavailable until a barrier is reached**.

Procedures that require the complete active point set, final reconstruction parameters, "GPU" visibility, or "codec" state remain downstream:

```
final geometric scaling
pose transformation
projection-specific transformed bounds
6-view projection
visibility / depth conflict resolution
"Geometry" / "Texture" / "Occupancy" "Atlas" packing
"I420" generation
"H.265" / "NVENC" compression
```

The `Encoder` is neither a conventional isolated application nor a pure stateless data-plane function. It still contains a frame-completion bound before projection, but it performs packet-by-packet conversion & placement as elements arrive, consumes progressive / final geometric metadata, cooperatively services "DPDK" while "CPU" / "GPU" work is in progress, & decouples "codec" input through a writer queue. It can be interpreted as a **hybrid frame-aware elaboration node** positioned between application-level semantics & data-plane-oriented incremental execution.

### 1.3 Connection with the Reference Pipeline

The methodical approach is informed by the state-of-the-art literature, including baseline & works already cited in this repository, which provides a complete volumetric streaming mechanism & a useful performance-evaluation framework.

The present project does **not** reproduce that architecture verbatim. Conversely, it reformulates the same workload around:

```
"DPDK"-native packet I / O
"OVS"-"DPDK" switching
explicit service chaining
"NSH"-style "SPI" / "SI" steering
in-place geometric aggregation
"GPU" projection
persistent "FFmpeg" / "NVENC" encoding
"codec" pre-roll
frame-aware "MPEG-TS" attribution
temporal source regulation
per-node telemetry
```

Consequently, comparisons with the reference implementation must be made only at **semantically equivalent frontiers**. Precise numerical equality is neither expected nor methodologically correct when transport mechanisms, buffering, node boundaries, "CPU" placement, cache residency, "GPU" kernels, or "codec" queuing policy differ.

### 1.4 Guidelines & "Codec" Scope

Two distinctions remain essential for the interpretation of this work.

First, the project adopts the **"SFC" concepts** of "Service Path Identifier" ( "SPI" ) & "Service Index" ( "SI" ), , alogside with an experimental "MD-Type-2"-like context layout. The wire representation is nevertheless a closed project protocol & is described as **"NSH"-inspired**, not as a claim of generic "RFC 8300" interoperability.

Second, `Encoder` does **not** implement "MPEG" "V-PCC" or "G-PCC". It constructs a custom 6-view "Geometry" / "Texture" / "Occupancy" "Atlas" & uses "HEVC" ( `hevc_nvenc` ) as the video compression engine for that representation. Its bitrate & latency therefore characterise a property of the projection-&-video path & must not be reported as a standards-compliant coding result.

---

## 🧩 2. Architectural Roles & Why Each Node Matters

| Device | Importance |
|---|---|
| `Camera` | Establishes the absolute timeline, reads the prepared fixed-width frames, serialises coordinates into network byte order, packetises below the "MTU", & applies the most recent `temporal_skip` before a frame enters the chain. Thus, the source prevents unnecessary elements from consuming downstream resources once the workload controller requests temporal relief. |
| `SFF1` | Demonstrates the primary in-path computation principle. It computes progressive centroid / extent / bounding-box information during forwarding & the exact final `max_r` at frame completion, then exports the result through experimental service metadata. It also decapsulates the "Temporal" control chain but no longer decides which source frames are skipped. |
| `SFF2` | Separates traffic steering from service-function awareness. It acts as a 4-port forwarder & stateful "NSH" proxy, enforcing the primary & both reverse control paths. Capable boundaries use "SPI" / "SI" semantics, whereas Encoder & the future Decoder operate on ordinary "UDP" application packets. |
| `Encoder` | Reconstructs complete frames while performing packet-arrival conversion incrementally, validates & consumes the cumulative metadata produced by SFF1 when offload is enabled, falls back to local calculations otherwise, executes the "CUDA" projection pipeline, feeds a persistent pre-rolled "NVENC" process, attributes asynchronous "MPEG-TS" output to source elements, & regulates temporal load according to its measured processing condition. |

"OVS"-"DPDK" stays deliberately immediate. It provides deterministic adjacency between "vhost-user" interfaces & a "Default-Deny" policy substrate. It does not execute point-cloud computations. Service functions are the components responsible for application-specific data-plane work.

The architectural boundary is therefore explicit:

```
"OVS"-"DPDK"      -> deterministic virtual adjacency
"SFF1" / "SFF2" -> route computation & steering
"Encoder"       -> hybrid frame / "GPU" / "codec" processing
"Camera"        -> workload admission through temporal selection
```

---

## 🔗 3. Service-Chain Semantics

### 3.1 "Main" Service Path

The project uses:

```
MAIN_SPI = 100
```

The "SI" model is kept by `SFF2` even when an adjacent application function is "SFC"-unaware. Primary transitions are:

```
SFF1 emits aware state                         : "SPI 100", "SI 255" + geometry context
SFF2 captures state & removes service envelope : Encoder receives plain "UDP" + geo_agg_hdr
Encoder returns plain compressed traffic       : proxy state advances "SI 255 -> 254"
SFF2 forwards plain application traffic         : Decoder remains "SFC"-unaware
Future Decoder return                           : proxy state advances "SI 254 -> 253"
SFF2 re-imposes base "NSH" toward SFF3          : "SPI 100", "SI 253"
```

Accordingly, the complete validated portion is:

```
Camera -> SFF1 ( "GAC", aware boundary )-> SFF2 ( proxy capture / decapsulation ) -> Encoder ( unaware ) -> SFF2 ( proxy state transition ) -> Decoder ( unaware )
```

`SFF2` ( Route 2 ) contains the fundamental scaffold required to insert the service header toward `SFF3`, but `Decoder`-side packet contract is not committed. Semantics & telemetry are therefore intentionally deferred rather than implied from an unstable format.

### 3.2 "Temporal" Service Path

"Temporal" adaptation uses:

```
TEMPORAL_SPI = 200
TEMPORAL_SI  = 255
```

`Encoder` generates this decision, within the present scenario, the dominant modification target is **elaboration capacity**, not user-selected visual quality.

```
Encoder
  -> standard "UDP" payload
  -> SFF2 classifies + imposes service metadata
  -> SFF1 validates + removes the outer encapsulation
  -> ...
  -> Camera updates temporal skip
```

The structure is:

```
frame_id : uint32
skip     : uint16
padding  : uint16
```

Once the `Camera` has adopted a value, it selects source frames according to the active factor before segmentation. The resulting nominal active-element rate is:

```
FPS = TARGET_FPS / skip
```

Such organization matters. If an overloaded `Encoder` were to discard frames only after they crossed `Camera`, `SFF1`, & `SFF2`, upstream bandwidth & computation would already have been consumed. Returning the decision to the `Camera` switches the controller into an admission mechanism for the next part of the stream.

### 3.3 "Pose" Service Path

"Pose" control is separated from temporal regulation:

```
POSE_SPI = 300
POSE_SI  = 255
```

The intended direction is:

```
User -> SFF3 -> SFF2 -> Decoder
```

At the `SFF2` frontier, an "NSH"-encapsulated pose payload is stripped & redirected as plain "UDP" to `Decoder`. The current upstream run does not exercise this chain because `Decoder`, `SFF3`, & `User` interaction are still under development.

This is also why `yaw`, `pitch`, & `zoom` remain static in the present `Camera` to `Encoder` path. Dynamic user stance is planned to become a **downstream reconstruction or rendering concern**.

### 3.4 Protocol Clarification

The project uses an 8-byte `nsh_hdr`, the "SPI" / "SI" concept, a "Time-to-Live" ( "TTL" ) field, & a defined context for geometric metadata. `SFF2` also preserves proxy state while service functions are unaware of the chain envelope.

Nevertheless, the realization must still be described as **experimental / "NSH"-inspired**, not as a generic "RFC 8300" interoperable stack. The spatial container is a fixed contract shared by the participating programs, & the selected `next_protocol` / information conventions are interpreted inside this repository.

The service-chain-facing endpoints predominantly use the project-selected "UDP" port `6633`, while the source-facing Camera adjacency retains its dedicated ports. The current address contract is:

| Adjacency / Endpoint | "IPv4" Address | "UDP" Port |
|---|---|---:|
| `Camera` | `10.0.0.2` | `49432` |
| `SFF1` Camera-facing endpoint | `10.0.1.254` | `5001` |
| `SFF1` -> `SFF2` endpoint | `10.0.2.1` | `5001` |
| `SFF2` -> `SFF1` endpoint | `10.0.2.2` | `6633` |
| `SFF2` -> `Encoder` endpoint | `10.0.3.1` | `6633` |
| `Encoder` -> `SFF2` endpoint | `10.0.3.2` | `6633` |
| `SFF2` -> `Decoder` endpoint | `10.0.4.1` | `6633` |
| future `Decoder` -> `SFF2` endpoint | `10.0.4.2` | `6633` |
| `SFF2` -> `SFF3` endpoint | `10.0.5.1` | `6633` |
| future `SFF3` -> `SFF2` endpoint | `10.0.5.2` | `6633` |

These addresses are an explicit closed-testbed contract rather than a dynamically routed deployment model. "OVS"-"DPDK" establishes the physical virtual adjacency, while the native functions construct / validate the corresponding "Ethernet" / "IPv4" / "UDP" envelope.

Whenever an "IPv4" header is created or rewritten, its header checksum is cleared & recomputed with `rte_ipv4_cksum()`. The current "IPv4" experiment deliberately writes the "UDP" checksum field as zero. This is valid for the present "IPv4" testbed, but it must not be carried unchanged into a future "IPv6" design, where a zero "UDP" checksum is not the ordinary endpoint rule.

The important aspect at this stage is the ability to evaluate conditions, in-path elaboration, proxy decapsulation / re-encapsulation, & "SFC"-unaware components under one controlled protocol definition.

---

## 📦 4. Data Representation & Packet Formats

### 4.1 Endianness & Portability

Offline ( `.bin` ) & network ( live ) representations intentionally have different roles.

The `Converter.py` script writes a contiguous 16-byte point data type using "Little-Endian" `float32` coordinates:

```
"x", "y", "z" -> "<f4"
"r", "g", "b" -> "u1"
"padding"     -> "u1"
```

Before network transmission, `Camera` reinterprets each "IEEE-754" value as a 32-bit word & converts the bit pattern to network byte order. `SFF1` reverses that operation for geometric computation & re-encodes coordinates when exporting metadata. `Encoder` performs the network-to-host shift when receiving the point stream & spatial context.

Consequently:

- protocol fields are serialised on the "DPDK" path;
- geometric details use the same explicit bit-pattern convention;
- the file remains a host-preparation artefact using the documented layout.

This removes the previous ambiguity between storage & network formats. The on-wire representation is explicit for implemented components, while the offline result stays tied to the Converter agreement used by this experiment.

### 4.2 Common Structures

| Structure | Size | Function |
|---|---:|---|
| `point_tx` | `16 B` | `x`, `y`, `z` as `float32`, "RGB" as `uint8`, plus an additional `padding` byte |
| `cam_hdr` | `40 B` | Frame identity, packet sequence, Camera timestamp, static pose reference, temporal skip, original-point count, & points in the packet |
| `nsh_hdr` | `8 B` | Service-chain base carrying "SPI", "SI", "TTL", metadata type, & next-protocol fields |
| `nsh_md2_ctx_hdr` | `4 B` | Project geometric-context descriptor |
| `geo_agg_hdr` | `44 B` | Centroid, extent, bounding-box centre, precise / progressive `max_r`, & active-point count |
| `enc_hdr` | `48 B` | Media packet identifier, scale, pose-compatibility, & reconstruction information |
| `temporal_payload` | `8 B` | Source frame associated with the control decision & requested adjustment |
| `pose_payload` | `12 B` | `yaw`, `pitch`, & `zoom` values for User to Decoder course |

The project no longer uses the previous fixed `int_hdr` sums / extrema block as its public downstream geometry contract. `SFF1` sends directly usable quantities through `geo_agg_hdr`.

### 4.3 Point Record — 16 Bytes

```
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

The explicit padding byte creates a deterministic 16-byte record that is shared by the Converter & native devices. The layout is convenient for fixed-offset packet parsing & aligned host-side storage. However, it is not, by itself, a claim that every operation is vectorised ( e.g., via "SIMD" executions ).

### 4.4 Camera Header

The 40-byte `cam_hdr` survives the upstream point path & carries out frame-local details:

```
frame_id
sequence_number
timestamp
yaw
pitch
zoom
temporal_skip
original_points
points_in_packet
padding
```

`frame_id` identifies the original source frame & remains meaningful even when temporal selection removes intermediate elements. `sequence_number` represents the `Camera` packet position. `original_points` defines the complete expected size & `points_in_packet` records the contribution of the current datagram, allowing each receiver to reconstruct them without any derivation from an external index.

The initial pose, as stated various times, is deliberately static:

```
yaw   = 0.0
pitch = 0.0
zoom  = 1.0
```

Dynamic stance is reserved for the independent "Pose" service path.

### 4.5 Geometric Metadata Produced by SFF1

The "GAC" exports:

```
centroid_x, centroid_y, centroid_z
extent_x, extent_y, extent_z
bbox_center_x, bbox_center_y, bbox_center_z
max_r
active_point_count
```

For an observed point prefix `P_N`, the progressive quantities are:

```
C_N = ( 1 / N ) * sum_{ p in P_N }( p )
E_N = p_max,N - p_min,N
B_N = ( p_min,N + p_max,N ) / 2
```

Middle elements carry the best currently available snapshot. On the frame-completing packet, `C_N`, `E_N`, & `B_N` become final & `max_r` is computed exactly as:

```
max_r = max_{ p in P_frame } || p - C_final ||_2
```

`active_point_count` allows `Encoder` to determine whether the received information describes the complete active point set. With `OFFLOAD_MODE_ENABLED`, valid metadata eliminates local geometry aggregation & radius computation.

### 4.6 Encoder Structure

The 48-byte `enc_hdr` accompanies compressed media leaving the node:

```
frame_id
packet_id
global_scale
box_center_x
box_center_y
box_center_z
yaw
pitch
final_scale
centroid_x
centroid_y
centroid_z
```

`packet_id` restarts for each encoded frame attributed from the "MPEG-TS" / "PES" stream. The remaining fields preserve the geometric quantities required by `Decoder` to obtain the projected representation.

`yaw` & `pitch` are currently retained for packet-format compatibility but remain static in the validated run. Dynamic pose information is expected to arrive independently through the "Pose" service-chain.

---

## 📐 5. "MTU"-Aware Packet Design

The project designs every normal packet around a standard:

```
"IPv4" "MTU" = 1500 B
```

All implemented application packets are sized so the complete datagram remains below this limit. "Eth" framing is shown separately because the "MTU" excludes its 14-byte header.

### 5.1 Camera -> SFF1

`Camera` sends up to:

```
POINTS_PER_PACKET = 80
POINT_SIZE_BYTES  = 16
```

Therefore:

```
Point payload = 80 x 16 = 1280 B
```

```
"IPv4"                     20 B
"UDP"                       8 B
cam_hdr                    40 B
80 * point_tx            1280 B
-------------------------------
"IPv4" datagram          1348 B
"Eth" frame              1362 B   ( excluding "FCS" / preamble )
"IP"-"MTU" margin         152 B
```

The value `80` is intentionally conservative because the same point packet will receive the additional service metadata inserted by `SFF1`, thereby leaving deliberate headroom rather than filling the "MTU" to the theoretical maximum.

### 5.2 SFF1 -> SFF2 -> Encoder

`SFF1` removes the previous "Eth" / "IPv4" / "UDP" network envelope, performs spatial aggregation, & prepends its own service block:

```
"IPv4"                     20 B
"UDP"                       8 B
nsh_hdr                     8 B
nsh_md2_ctx_hdr             4 B
geo_agg_hdr                44 B
cam_hdr                    40 B
80 * point_tx            1280 B
-------------------------------
"IPv4" datagram          1404 B
"Eth" frame              1418 B
"IP"-"MTU" margin          96 B
```

At Route 0, `SFF2` acts as an "NSH" proxy. It strips the `nsh_hdr` + context encapsulation prior to unaware `Encoder` while **preserving ****`geo_agg_hdr`**** as application-visible geometry**:

```
"IPv4"                     20 B
"UDP"                       8 B
geo_agg_hdr                44 B
cam_hdr                    40 B
80 * point_tx            1280 B
-------------------------------
"IPv4" datagram          1392 B
"Eth" frame              1406 B
"IP"-"MTU" margin         108 B
```

The metadata is therefore present where service-path processing requires it but does not leak into "SFC"-unaware parsers.

### 5.3 Encoder -> SFF2 -> Decoder

The `Encoder` groups complete "MPEG-TS" without tearing them across various datagrams. Packets are fixed at:

```
TS_PACKET_SIZE   = 188 B
MTU_PAYLOAD_SIZE = 7 * 188 = 1316 B
```

Largest encoded-media size is:

```
"IPv4"                      20 B
"UDP"                        8 B
cam_hdr                     40 B
enc_hdr                     48 B
7 * "MPEG-TS"             1316 B
--------------------------------
"IPv4" datagram           1432 B
"Eth" frame               1446 B
"IP"-"MTU" margin           68 B
```

Hence:

```
7 x 188 = 1316 B  -> valid
8 x 188 = 1504 B  -> exceeds the available 1376 B
```

`Encoder` & `Decoder` are "SFC"-unaware. "SFF2" retains the primary service-path state around those functions rather than forcing each application parser to implement service-chain headers.

### 5.4 "Temporal" & "Pose" Packets

Control paths use intentionally small payloads. For the "Temporal" chain,

`Encoder` -> `SFF2` ( plain ):

```
"IPv4" + "UDP" + temporal_payload = 20 + 8 + 8 = 36 B
```

`SFF2` -> `SFF1` ( experimental "NSH" ):

```
"IPv4" + "UDP" + nsh_hdr + temporal_payload = 20 + 8 + 8 + 8 = 44 B
```

`SFF1` -> `Camera` ( plain ) returns to the `36 B` "IPv4" datagram.

"Pose" chain supports a 12-byte element structure:

```
SFF3 -> SFF2    : 20 + 8 + 8 + 12 = 48 B "IPv4" datagram
SFF2 -> Decoder : 20 + 8 + 12     = 40 B ...
```

Remaining application formats are **not specified yet**. These should be documented only when `Decoder` output contract is stable.

### 5.5 "Virtio" Queue Size & "DPDK" Rx / Tx Dimensioning

The project distinguishes the **"virtio-user" queue capacity configured at "vdev" creation** from the **descriptor count requested when the application sets up each "ethdev" Rx / Tx queue**. Such elements are related controls rather than additive buffering layers. For a "virtio-user" "PMD", the "ethdev" queues are backed by "virtqueues", & the descriptor count requested by the application must remain compatible with the capabilities exposed by the "virtio-user" / "vhost" path. They should therefore be interpreted together, not summed as if they were independent cascaded queues.

**"virtio-user" queue size**

The "EAL" `queue_size` parameter configures the depth of the "virtio-user" "virtqueue" associated with the "vhost-user" transport. A larger value increases the number of descriptors that can remain outstanding between a service function & the "OVS"-"DPDK" "vhost" endpoint before descriptor exhaustion becomes visible as backpressure.

```
Camera                  Rx 4096 / Tx 4096
SFF1 Camera-facing      Rx 4096
SFF1 SFF2-facing        Rx 1024
SFF1                    Tx 1024
SFF2 Encoder-facing     Rx 4096
SFF2 Decoder-facing     Rx 4096
SFF2 SFF1/SFF3-facing   Rx 1024
SFF2                    Tx 1024
Encoder                 Rx 4096 / Tx 4096
```

The asymmetry is intentional. Links attached to compute-heavy end functions are provisioned with additional elasticity, whereas relay-to-relay interfaces remain shallower because both sides are expected to return to their polling loops continuously.

These queues absorb finite producer / consumer skew, but the source can still encounter local Tx-ring congestion. For this reason, telemetry distinguishes actual `rte_eth_tx_burst()` execution from wall-clock submission time & records zero-accept / resubmission behaviour explicitly.

Chosen depths also interact with the shared `mbuf` pools. Each active node uses:

```
NUM_MBUFS       = 16383
MBUF_CACHE_SIZE = 256
```

& `SFF2`, for instance, requests a total of:

```
1024 + 4096 + 4096 + 1024 = 10240 Rx descriptors
```

across its four ports. The pool therefore retains headroom for in-flight packets, burst processing, & per-"lcore "caching rather than being dimensioned only to the sum of the receive rings.

Larger queue capacities are not universally better. They consume additional memory, can postpone the visibility of a persistently overloaded consumer, & may increase the amount of traffic waiting in the path before backpressure becomes observable. Descriptor depth, `queue_size`, core placement, & cooperative polling must therefore be treated as a **joint buffering-&-scheduling design**, & all of them must remain fixed when comparing benchmark runs.

---

## 📷 6. Camera — Scheduling, Warm-Mode, "Temporal" Selection, & Telemetry

### 6.1 Role

The `Camera` is the only node that originates the volumetric schedule. It reads the pre-converted "Loot" sequence, assigns source frame IDs, serialises coordinates for network transmission, packetises each point cloud, timestamps the element, & submits "DPDK" bursts to "SFF1". Consequently, its contribution is both functional & experimental. Every downstream timing quantity is ultimately conditioned by the source cadence, burst structure, cache mode, & residency policy established at this component.

The reference workload is configured as:

```
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
| `CACHE_MODE_WORST` | Inside frame loop | Inside frame loop | Deliberately pessimistic mode including allocation & file read in the frame path |

The current source selects:

```
CACHE_MODE = CACHE_MODE_MIDDLE
WARM_MODE  = WARM_MODE_ENABLED
```

`WARM_MODE_ENABLED` maps & locks source documents before the measured sequence so the timed `fread()` path retains ordinary file-read semantics while operating over a resident file-backed working set. The resulting `disk_io_ms` should be interpreted as **timed buffered acquisition from the warmed condition**, not as a direct measurement of cold physical-storage latency.

Results obtained with different settings are different experimental conditions & must not be mixed in the same performance claim.

### 6.3 Packetisation

Each selected frame is divided into packets of at most 80 points. The `Camera` assigns:

```
frame_id
sequence_number
timestamp = t_send_start
temporal_skip
original_points
points_in_packet
```

The `Camera` timestamp is generated immediately before the packet-transmission loop & propagated unchanged across all packets of the same frame.

Coordinates are converted from the prepared little-endian host representation to network-order "IEEE-754" bit patterns before transmission. "RGB" & the explicit padding byte remain byte-valued fields.

### 6.4 Isochronous Scheduling & Camera-Side "Temporal" Selection

The source establishes one absolute session origin & computes the ideal deadline for frame index `k` as:

```
T_ideal( k ) = T_0 + k * ( 1 / TARGET_FPS )
```

For the current configuration:

```
TARGET_FPS = 30
T_base     = 33.333... ms
```

Before each source-frame decision, the `Camera` polls for a `temporal_payload`. If the current factor is `s`, the frame is selected when:

```
( frame_id - 1 ) mod s = 0
```

and the nominal admitted rate becomes:

```
FPS_effective = TARGET_FPS / s
```

Selected & skipped frames both advance against the same absolute source timeline. This prevents temporal adaptation from redefining the session clock & preserves meaningful frame-ID-based scheduling & jitter calculations downstream.

The current design therefore has no separate `PACING_MODE`. Source timing is governed by the absolute target schedule, while local Tx-ring pressure is exposed through retry telemetry rather than hidden behind an additional pacing heuristic.

### 6.5 Meaning of Tx Backpressure Counters

A zero return from `rte_eth_tx_burst()` means that the local Tx path accepted no packets in that attempt. The `Camera` retries with a bounded pause strategy & records:

```
tx_zero_accepts
tx_partial_accepts
tx_resubmit_calls
tx_resubmitted_packets
mbuf_starvation
```

These quantities must not be called "UDP" retransmissions. They occur **before successful local "DPDK" queue acceptance** & therefore measure producer / consumer pressure at the local packet-I/O boundary.

The validated run exhibits many zero-accept attempts because approximately ten thousand point packets are emitted per frame, yet every frame remains complete & `mbuf_starvation = 0`. The counters are therefore useful evidence that backpressure existed without being converted into application-visible loss in this experiment.

### 6.6 Camera Telemetry — Complete Semantics

The `Camera` exports **all 23 fields** shown below. Their boundaries deliberately separate logical work, local "DPDK" queue activity, & the source-residency interval; consequently they should not be collapsed into one generic transmission metric.

| Metric | Unit / Type | Exact Definition | Interpretation |
|---|---|---|---|
| `frame_id` | identifier | Original source-frame identifier ( `1 ... K_FRAMES` ). | Preserves the source timeline even when later `temporal_skip > 1` causes a frame to be omitted. |
| `status` | boolean | `1` when all declared points are accepted for local "DPDK" transmission & no `mbuf` allocation failure occurs; otherwise `0`. | A temporally non-selected source frame is also recorded with `status = 0`, but it is intentionally absent from the transmitted path rather than lost. |
| `current_skip` | factor | Snapshot of the active Camera-side `temporal_skip` sampled before the frame-selection decision. | Defines the effective admitted source rate as `TARGET_FPS / current_skip`. |
| `last_control_frame` | identifier | Frame identifier carried by the most recent admissible "Temporal" control decision received from `SFF1`. | Makes controller propagation observable at the source. |
| `timestamp_start_tx` | seconds | `t_send_start / timer_hz`; the Camera timestamp copied into `cam_hdr.timestamp`. | Shared-host reference used by downstream latency metrics. |
| `tx_points` | points | Sum of point records belonging to packets successfully accepted by the Camera-facing "DPDK" Tx queue. | Compared with the source frame population to determine `status`. |
| `tx_packets` | packets | Number of "DPDK" datagrams successfully accepted for the frame. | Approximately `ceil( tx_points / POINTS_PER_PACKET )` in a complete frame. |
| `payload_bytes` | bytes | `tx_points * POINT_SIZE_BYTES`. | Point bytes only; excludes `cam_hdr` & repeated network headers. |
| `internal_throughput_mbs` | MB/s | `( payload_bytes + sizeof( cam_hdr ) ) / 1e6 / send_duration_s`. | Decimal logical frame throughput over the Camera submission interval. |
| `logical_bitrate_mbps` | Mbit/s | `logical_frame_bytes * 8 * ( TARGET_FPS / current_skip ) / 1e6`. | Counts point payload plus one logical `cam_hdr` per frame. |
| `network_bitrate_mbps` | Mbit/s | `( payload_bytes + tx_packets * sizeof( main_hdr ) ) * 8 * effective_fps / 1e6`. | Includes repeated "Eth" / "IPv4" / "UDP" / `cam_hdr` overhead for each Camera packet. |
| `disk_io_ms` | ms | Timed acquisition interval for the selected `CACHE_MODE`. | With `CACHE_MODE_MIDDLE` + `WARM_MODE_ENABLED`, this is warmed buffered `fread()` latency, not cold physical-storage latency. |
| `serialization_ms` | ms | Time required to convert point coordinates into the explicit on-wire network byte order. | Measures point-record serialisation only. |
| `tx_duration_ms` | ms | `t_send_end - t_send_start`. | Wall-clock packet-submission interval, including local zero-accept retry pauses. |
| `active_tx_ms` | ms | Sum of intervals spent inside `rte_eth_tx_burst()` calls. | Does not include backoff / pause time between calls. |
| `active_process_ms` | ms | `disk_io_ms + serialization_ms + tx_duration_ms`. | Reference-compatible active Camera processing boundary. |
| `total_residency_ms` | ms | `t_send_end - t_start_residency`, where residency begins before timed file acquisition. | Complete Camera frame residence for a selected source frame. |
| `node_efficiency_pct` | % | `100 * active_process_ms / total_residency_ms`. | Expected to approach `100 %` because the Camera residence is intentionally bounded around its own active source path. |
| `tx_zero_accepts` | count | Number of Tx attempts for which `rte_eth_tx_burst()` returns `0`. | Local queue backpressure indicator; **not** a "UDP" retransmission count. |
| `tx_partial_accepts` | count | Number of Tx attempts accepting fewer packets than requested while accepting at least one. | Separates partial progress from total zero acceptance. |
| `tx_resubmit_calls` | count | Number of subsequent Tx calls performed after a preceding incomplete acceptance. | Counts local re-presentation attempts. |
| `tx_resubmitted_packets` | packet-attempts | Sum of packet requests presented by resubmission calls. | The same unsent `mbuf` may contribute repeatedly; therefore this value is intentionally larger than the unique packet population. |
| `mbuf_starvation` | count | Frame-local `mbuf` allocation failures encountered while packetising. | A non-zero value may make the frame incomplete even if the transport itself is error-free. |

The principal equations are therefore:

```text
logical_frame_bytes = payload_bytes + sizeof( cam_hdr )
network_frame_bytes = payload_bytes + tx_packets * sizeof( main_hdr )
effective_fps       = TARGET_FPS / current_skip

internal_throughput_mbs = ( logical_frame_bytes / 1,000,000 ) / tx_duration_s
logical_bitrate_mbps    = logical_frame_bytes * 8 * effective_fps / 1,000,000
network_bitrate_mbps    = network_frame_bytes * 8 * effective_fps / 1,000,000

active_process_ms    = disk_io_ms + serialization_ms + tx_duration_ms
node_efficiency_pct  = 100 * active_process_ms / total_residency_ms
```

A particularly important distinction concerns Tx resubmission. `tx_resubmitted_packets` is an **attempt-volume** counter, not a unique packet counter: if the same pending `mbuf` is presented several times while the virtual Tx path rejects it, it contributes repeatedly. This is why the value can legitimately exceed `tx_packets` by a large factor without indicating duplicated wire traffic.

### 6.7 "End-of-Stream" Behaviour

After the configured sequence, the `Camera` emits repeated `END_OF_STREAM` control packets containing the `Camera` header & no point payload. Redundancy is used to make the terminal condition robust to transient local queue behaviour in the experimental environment.

The "EOS" marker is a protocol event, not an additional source frame, & is excluded from the 300-frame telemetry table.

---

## 🧠 7. SFF1 — "Geometry-Aware Classifier" ( "GAC" ) & In-Path Aggregation

### 7.1 Role

`SFF1` is the "Geometry-Aware Classifier" ( "GAC" ) of the current architecture. Its purpose is to demonstrate that useful frame geometry can be derived **while point packets are already traversing the service path**, instead of reconstructing the same statistics for the first time inside the `Encoder`.

For each valid `Camera` packet it:

```text
validates "Ethernet" / "IPv4" / "UDP" and Camera metadata
decodes point coordinates from network byte order
updates frame-progressive geometric state
stores "XYZ" in a preallocated frame-local workspace
computes progressive centroid / extent / bounding-box centre
computes exact max_r when the frame becomes complete
replaces the outer network envelope in place
adds experimental "NSH" + geometric context
forwards the original Camera application payload
records frame-level telemetry
```

The "GAC" is intentionally more than a forwarding label while remaining substantially narrower than a complete application processor.

### 7.2 "Temporal" Control Is Relayed, Not Decided, by SFF1

The current `SFF1` does **not** perform source-frame temporal filtering.

"Temporal" adaptation is generated by the `Encoder` & applied by the `Camera`. `SFF1` is only the final service-chain-aware relay in the reverse "Temporal" path:

```text
SFF2 -> SFF1 : "NSH"-encapsulated temporal_payload
SFF1         : validates "SPI" = 200 / "SI" = 255
SFF1         : removes the service-chain envelope
SFF1 -> Camera: plain "UDP" temporal_payload
```

The `current_skip` observed by `SFF1` on the primary data path is therefore the factor already selected by the `Camera` & carried inside `cam_hdr`. It is used for telemetry & effective-rate interpretation, not for a second local frame-drop decision.

### 7.3 Progressive Geometric Offloading

For the active prefix containing `N` points, `SFF1` maintains:

```text
S_N     = sum_i( p_i )
p_min,N = component-wise min_i( p_i )
p_max,N = component-wise max_i( p_i )
```

and derives:

```text
C_N = S_N / N
E_N = p_max,N - p_min,N
B_N = ( p_min,N + p_max,N ) / 2
```

These operations can be updated as each point arrives & therefore fit naturally inside a data-plane-oriented classifier.

The exact farthest-point radius is different:

```text
max_r = max_i || p_i - C_final ||_2
```

Since `C_final` changes until the final point is known, exact `max_r` cannot generally be committed from the first packet without either revisiting previous points or moving the computation to an upstream preprocessing stage. The current implementation therefore stores only `XYZ` coordinates in a preallocated workspace & performs a second radius pass once the frame is complete.

This is a deliberate compromise. The expensive application representation is not rebuilt inside `SFF1`, packet forwarding continues while the first pass is performed, & downstream `Encoder` geometry work is removed when the final metadata are valid.

### 7.4 In-Place Header Replacement

The incoming `Camera` datagram contains:

```text
[ "Ethernet" | "IPv4" | "UDP" | cam_hdr | points ]
```

`SFF1` removes the original network envelope & prepends:

```text
[ "Ethernet" | "IPv4" | "UDP" | nsh_hdr | md2_ctx | geo_agg_hdr ]
```

while preserving:

```text
[ cam_hdr | points ]
```

The resulting packet is:

```text
[ "Ethernet" | "IPv4" | "UDP" | nsh_hdr | md2_ctx | geo_agg_hdr | cam_hdr | points ]
```

This is the central in-path transformation: service metadata are added to the packet already being forwarded rather than creating a separate frame-level side channel.

### 7.5 Frame-Global Boundary Constraint

There are two distinct meanings of a geometric "boundary" in the present implementation.

Axis-aligned extrema are packet-progressive:

```text
min_x / max_x
min_y / max_y
min_z / max_z
```

and therefore become increasingly accurate as packets arrive.

A centroid-dependent radial boundary is not packet-final until the centroid is final. Similarly, a boundary after an arbitrary future user pose would require that pose before it could be evaluated. No packet format can remove this mathematical dependency by itself.

The project therefore avoids an artificial preprocessing solution at the `Camera` or offline converter. Such a solution could certainly send the final geometry earlier, but it would invalidate the research question of performing the operation at the service function **in place on the data path**. The present design preserves that objective & makes the unavoidable frame-global step explicit.

### 7.6 SFF1 Telemetry — Complete Semantics

`SFF1` exports **36 fields**. The current version treats the node as an active "Geometry-Aware Classifier" ( "GAC" ), hence geometry, exact radius, receive span, active forwarding work, & complete residence remain independently observable.

| Metric | Unit / Type | Exact Definition | Interpretation |
|---|---|---|---|
| `frame_id` | identifier | Original Camera frame identifier. | Maintains source-relative timing semantics. |
| `status` | boolean | `1` when the full declared point population is received & forwarded; otherwise `0`. | Functional integrity flag for the "GAC" output. |
| `current_skip` | factor | `temporal_skip` copied from the Camera header for the current frame. | `SFF1` observes this value; it no longer chooses source frames. |
| `camera_send_timestamp` | seconds | Camera `cam_hdr.timestamp` converted from timer cycles. | Absolute shared-host source reference. |
| `recv_start_timestamp` | seconds | First accepted packet-arrival timestamp for the frame. | Defines node entry. |
| `node_exit_timestamp` | seconds | Timestamp selected for frame completion, normally the last successful Tx activity. | Defines node exit for residency & scheduling analysis. |
| `original_points` | points | Point population declared by `cam_hdr.original_points`. | Reference denominator for integrity. |
| `rx_points` | points | Total point records validated at `SFF1` ingress. | Must equal `original_points` for a complete frame. |
| `tx_points` | points | Point records associated with successfully accepted outgoing packets. | Must equal `rx_points` for `status = 1`. |
| `rx_packets` | packets | Validated Camera packets received for the frame. | Input segmentation indicator. |
| `tx_packets` | packets | Packets successfully forwarded toward `SFF2`. | Output segmentation is expected to preserve the Camera packet count. |
| `payload_bytes` | bytes | `tx_points * sizeof( point_tx )`. | Forwarded point bytes only. |
| `data_integrity_pct` | % | `100 * rx_points / original_points`. | Point-population integrity at `SFF1` ingress. |
| `internal_throughput_mbs` | MB/s | `( rx_points * 16 + one cam_hdr ) / 1e6 / receive_s`, where `receive_s = last_rx - first_arrival`. | Pure ingress-span logical throughput rather than residency-normalised throughput. |
| `logical_bitrate_mbps` | Mbit/s | `( tx_point_bytes + one cam_hdr ) * 8 * effective_fps / 1e6`. | Logical forwarded workload at the active temporal rate. |
| `network_bitrate_mbps` | Mbit/s | `( tx_point_bytes + tx_packets * ( sizeof( main_hdr ) + sizeof( cam_hdr ) ) ) * 8 * effective_fps / 1e6`. | Includes repeated "Eth" / "IPv4" / "UDP" + experimental "NSH" / context / geometry metadata + `cam_hdr`. |
| `tx_duration_ms` | ms | `last_successful_tx - first_successful_tx`. | Wall span of successful egress activity. |
| `active_tx_ms` | ms | Accumulated time spent inside `rte_eth_tx_burst()` calls. | Local Tx execution only. |
| `active_process_ms` | ms | Accumulated packet-processing, geometry, envelope-rewrite & flush work attributed to the frame. | Numerator used by current `node_efficiency_pct`. |
| `geometry_aggregation_ms` | ms | Accumulated first-pass sum / extrema / progressive metadata computation. | Disjoint from the exact `max_r` pass in the current implementation. |
| `max_r_ms` | ms | Exact second-pass farthest-point radius calculation executed when the final point population is known. | Measures the frame-global geometric dependency intentionally offloaded to the "GAC". |
| `cycle_ms` | ms | `current_frame_exit - previous_frame_exit` ( first frame starts from its own initial arrival ). | Source-cycle occupancy reference. |
| `header_wait_ms` | ms | `max( 0, cycle_ms - total_residency_ms )`. | Inter-frame idle / waiting component outside the current frame residence. |
| `total_residency_ms` | ms | `frame_exit - first_arrival`. | Complete `SFF1` residence from first input packet to frame completion / egress. |
| `node_efficiency_pct` | % | `100 * active_process_ms / total_residency_ms`. | Fraction of residence directly attributed to active `SFF1` work. |
| `camera_to_node_latency_ms` | ms | `first_sff1_arrival - camera_send_timestamp` on the shared host timer domain. | Local Camera-to-`SFF1` propagation / scheduling interval. |
| `schedule_delay_ms` | ms | `( first_arrival - session_start ) - ( frame_id - first_arrival_frame_id ) / TARGET_FPS`, converted to ms. | Deviation of processing availability from the original isochronous Camera timeline. |
| `network_jitter_ms` | ms | Absolute difference between observed first-arrival spacing & the frame-ID-derived ideal spacing. | Remains meaningful when future temporal gaps make consecutive received IDs non-adjacent. |
| `eth_errors` | cumulative count | Snapshot of the process-wide invalid "Ethernet" counter when the frame record is initialised. | Cumulative diagnostic, **not** a frame-local delta. |
| `ipv4_errors` | cumulative count | Snapshot of the process-wide invalid "IPv4" counter. | Cumulative diagnostic. |
| `udp_errors` | cumulative count | Snapshot of the process-wide invalid "UDP" counter. | Cumulative diagnostic. |
| `nsh_errors` | cumulative count | Snapshot of experimental service-header / control-envelope validation failures. | Cumulative diagnostic. |
| `tx_zero_accepts` | count | Frame-local Tx attempts returning zero accepted packets. | Local "DPDK" backpressure. |
| `tx_partial_accepts` | count | Frame-local Tx calls with partial acceptance. | Local "DPDK" backpressure. |
| `tx_resubmit_calls` | count | Frame-local calls made after incomplete acceptance. | Local resubmission count. |
| `tx_resubmitted_packets` | packet-attempts | Requested packet population across resubmission calls. | May count the same pending packet more than once. |

The core byte / timing relations are:

```text
receive_s              = last_rx - first_arrival
logical_rx_frame_bytes = rx_points * 16 + one cam_hdr
logical_tx_frame_bytes = tx_points * 16 + one cam_hdr

effective_fps = TARGET_FPS / current_skip

internal_throughput_mbs = ( logical_rx_frame_bytes / 1,000,000 ) / receive_s
logical_bitrate_mbps    = logical_tx_frame_bytes * 8 * effective_fps / 1,000,000

network_tx_bytes = tx_points * 16 + tx_packets * ( "Eth" + "IPv4" + "UDP" + nsh_hdr + nsh_md2_ctx_hdr + geo_agg_hdr + cam_hdr )

network_bitrate_mbps = network_tx_bytes * 8 * effective_fps / 1,000,000

node_efficiency_pct = 100 * active_process_ms / total_residency_ms
```

`geometry_aggregation_ms` & `max_r_ms` are now intentionally non-overlapping. The former ends after the progressive sum / extrema update; only then does the frame-completing branch begin the exact centroid-dependent radius scan. This separation is necessary for a defensible measurement of the work moved from `Encoder` to the "GAC".


---

## 🔀 8. SFF2 — Multi-Port "NSH" Proxy & Service-Path Steering

### 8.1 Role & Ports

`SFF2` is the central four-port steering element:

```text
PORT_SFF1    = 0
PORT_ENCODER = 1
PORT_DECODER = 2
PORT_SFF3    = 3
```

Its current role is no longer adequately described as a simple forwarder. It is a **stateful experimental "NSH" proxy** that permits service-chain-aware functions & service-chain-unaware applications to coexist in the same path.

The three service-path identifiers are:

```text
"Main"     "SPI" = 100
"Temporal" "SPI" = 200
"Pose"     "SPI" = 300
```

### 8.2 Implemented Primary Routing

For `SFF1 -> Encoder`, `SFF2` receives the experimental "NSH" geometry envelope, validates the service state, decrements the "TTL", & stores frame-local proxy state. It then removes the incoming service-chain envelope & reconstructs a plain "Ethernet" / "IPv4" / "UDP" packet for the `Encoder`.

The preserved application content is:

```text
[ geo_agg_hdr | cam_hdr | point payload ]
```

The `Encoder` therefore consumes useful geometry without parsing "NSH".

When encoded media returns from the `Encoder`, `SFF2` identifies the `Encoder`-facing attachment as the next primary-chain transition, advances the proxy state, & forwards the plain application packet toward the future `Decoder`.

This design deliberately treats `Encoder` & `Decoder` attachment domains as **"SFC"-unaware application domains**: service-chain state is maintained by the proxy rather than duplicated inside the application.

### 8.3 "Temporal" & "Pose" Control Paths

The `Encoder` emits a plain 8-byte `temporal_payload`. `SFF2` recognises this control shape on the `Encoder` port & classifies it into:

```text
"SPI" = 200
"SI"  = 255
```

before forwarding it to `SFF1`.

The future pose direction works in the opposite encapsulation sense. A valid "Pose" packet from the `SFF3`-facing port uses:

```text
"SPI" = 300
"SI"  = 255
```

`SFF2` removes the service-chain envelope & forwards the 12-byte pose application payload as plain "UDP" to the future `Decoder`.

The two control paths are intentionally separate because they solve different problems:

```text
"Temporal" -> Encoder workload regulation -> Camera admission rate
"Pose"     -> User interaction / reconstruction state -> Decoder
```

### 8.4 Burst Ownership

`SFF2` aggregates outgoing packets into "DPDK" bursts. A burst is associated with the telemetry owner of the current frame & route so that successful Tx acceptance can be attributed to the correct frame.

This matters because a single physical `SFF2` worker services four logical ports & several packet classes. Without explicit burst ownership, Tx time & byte counts could be assigned to the wrong frame whenever routing transitions occur close together.

The implementation therefore flushes an existing burst when its owner or route changes & records first / last Tx cycles & active Tx cycles for the corresponding telemetry entry.

### 8.5 Proxy State & Unaware Service Functions

For each primary frame, `SFF2` stores a minimal proxy context containing:

```text
frame_id
"TTL"
"SI"
valid state
```

The state is captured when a valid `SFF1` primary packet enters the proxy boundary. While the packet is decapsulated for an unaware function, `SFF2` remains responsible for the logical service-index transition.

The design has two advantages for the experiment.

First, the `Encoder` can remain focused on point processing, "CUDA", & "codec" behaviour instead of becoming a second service-chain parser. Second, the same principle can be applied to the future `Decoder`, making it possible to compare application processing without requiring service-chain logic inside each application node.

The trade-off is explicit statefulness in the forwarder. Frame identity must remain valid & consistent for the proxy to advance or re-impose service state correctly.

The previous repository description of a `RED`-like / `AQM` heuristic does not apply to this snapshot. The current `SFF2` source contains no such congestion controller; workload adaptation has moved to the `Encoder`-driven "Temporal" path.

### 8.6 Route-Specific Payload Semantics

Route 0 ( `SFF1 -> Encoder` ) carries point data:

```text
logical payload -> point bytes + one cam_hdr
application context visible to Encoder -> geo_agg_hdr
```

Route 1 ( `Encoder -> Decoder` ) carries encoded media:

```text
logical payload -> "MPEG-TS" bytes + one cam_hdr + one enc_hdr
```

Route 2 ( `Decoder -> SFF3` ) is intentionally not assigned a stable application-byte formula yet. The base proxy branch can re-impose service metadata when the future `Decoder` returns a packet with a valid frame identifier, but the `Decoder` payload layout, completion semantics, & `SFF3` contract are not final.

This distinction is important: **proxy scaffolding exists for Route 2, but validated Route-2 telemetry does not**.

### 8.7 SFF2 Telemetry — Complete Semantics

`SFF2` uses one 36-column telemetry schema for its route arrays. In the current validated snapshot, quantitative records are meaningful for Route 0 ( `SFF1 -> Encoder` ) & Route 1 ( `Encoder -> Decoder` ). Route 2 keeps the proxy / forwarding scaffold but intentionally has no committed application-format accounting yet.

| Metric | Unit / Type | Exact Definition | Interpretation |
|---|---|---|---|
| `frame_id` | identifier | Source frame identifier carried by `cam_hdr`. | Used to bind route telemetry & per-frame proxy context. |
| `status` | boolean | Route 0: complete point reception + exact point forwarding. Route 1: non-empty media reception + exact media-byte forwarding. | Route-specific integrity indicator. |
| `current_skip` | factor | "Temporal" skip propagated in `cam_hdr`. | Defines `effective_fps = TARGET_FPS / current_skip`. |
| `camera_send_timestamp` | seconds | Original Camera timestamp retained across the "Main" path. | Cross-node reference. |
| `recv_start_timestamp` | seconds | First packet arrival at the selected `SFF2` route. | Node-entry reference. |
| `node_exit_timestamp` | seconds | Final route timestamp, normally last successful Tx / last relevant activity. | Node-exit reference. |
| `original_points` | points | Original point count retained from `cam_hdr`. | Meaningful on both routes as source metadata; Route 1 itself carries compressed media. |
| `rx_points` | points | Route 0 ingress point count. | Expected to be zero by design on Route 1. |
| `tx_points` | points | Route 0 successfully forwarded point count. | Expected to be zero by design on Route 1. |
| `rx_media_bytes` | bytes | Route 1 compressed-media bytes received after `cam_hdr + enc_hdr`. | Expected to be zero on Route 0. |
| `tx_media_bytes` | bytes | Route 1 compressed-media bytes associated with successful Tx packets. | Expected to equal `rx_media_bytes` for `status = 1`. |
| `rx_packets` | packets | Packets validated at route ingress. | Point datagrams on Route 0; compressed-media datagrams on Route 1. |
| `tx_packets` | packets | Packets successfully accepted on route egress. | Used for repeated wire-envelope accounting. |
| `payload_bytes` | bytes | Route 0: `rx_points * 16`. Route 1: `rx_media_bytes`. | Route-native application payload. |
| `data_integrity_pct` | % | Route 0: `100 * rx_points / original_points`. Route 1: `100 * tx_media_bytes / rx_media_bytes`. | Point-integrity vs byte-preservation semantics are intentionally route-specific. |
| `internal_throughput_mbs` | MB/s | `logical_rx_frame_bytes / 1e6 / receive_s`, with `receive_s = last_rx - first_arrival`. | Ingress logical throughput. |
| `logical_bitrate_mbps` | Mbit/s | `logical_tx_frame_bytes * 8 * effective_fps / 1e6`. | Counts application bytes + frame metadata once. |
| `network_bitrate_mbps` | Mbit/s | Route-specific outgoing bytes including the repeated Encoder- or Decoder-facing network envelope. | Represents the current "DPDK" datagram construction, not physical preamble / "FCS". |
| `tx_duration_ms` | ms | `last_successful_tx - first_successful_tx`. | Route egress span. |
| `active_tx_ms` | ms | Accumulated time spent inside route Tx-burst calls. | Local Tx execution. |
| `active_process_ms` | ms | Accumulated route classification, proxy processing, rewriting & Tx work. | Numerator of `node_efficiency_pct`. |
| `cycle_ms` | ms | `route_frame_exit - previous_route_frame_exit`. | Per-route source cycle. |
| `header_wait_ms` | ms | `max( 0, cycle_ms - total_residency_ms )`. | Inter-frame idle component for that route. |
| `total_residency_ms` | ms | `route_frame_exit - first_route_arrival`. | Complete frame residence inside the selected `SFF2` route. |
| `node_efficiency_pct` | % | `100 * active_process_ms / total_residency_ms`. | Active steering / proxy fraction of residence. |
| `camera_to_node_latency_ms` | ms | `first_route_arrival - camera_send_timestamp`. | Absolute shared-host Camera-relative latency; unlike the Encoder field, it is not baseline-corrected. |
| `schedule_delay_ms` | ms | First-arrival deviation from `( frame_id - first_arrival_frame_id ) / TARGET_FPS`. | Preserves the source timeline across temporal frame-ID gaps. |
| `network_jitter_ms` | ms | Absolute observed inter-arrival error relative to the frame-ID-derived expected interval. | Route-level timing dispersion. |
| `eth_errors` | cumulative count | Snapshot of process-wide "Ethernet" validation failures. | Cumulative diagnostic. |
| `ipv4_errors` | cumulative count | Snapshot of process-wide "IPv4" validation failures. | Cumulative diagnostic. |
| `udp_errors` | cumulative count | Snapshot of process-wide "UDP" validation failures. | Cumulative diagnostic. |
| `nsh_errors` | cumulative count | Snapshot of proxy / experimental "NSH" validation-state failures. | Cumulative diagnostic; can also reflect inconsistent per-frame proxy state. |
| `tx_zero_accepts` | count | Route-local Tx calls returning zero acceptance. | Local queue pressure. |
| `tx_partial_accepts` | count | Route-local partial Tx acceptances. | Local queue pressure. |
| `tx_resubmit_calls` | count | Route-local resubmission calls following incomplete Tx acceptance. | Not a transport retransmission. |
| `tx_resubmitted_packets` | packet-attempts | Packet requests presented by resubmission calls. | May recount a pending packet across multiple attempts. |

The route-specific formulas are:

```text
Route 0 ( SFF1 -> Encoder )

logical_rx_frame_bytes = rx_points * 16 + one cam_hdr
logical_tx_frame_bytes = tx_points * 16 + one cam_hdr

network_tx_bytes = tx_points * 16 + tx_packets * ( "Eth" + "IPv4" + "UDP" + geo_agg_hdr + cam_hdr )

integrity = 100 * rx_points / original_points

Route 1 ( Encoder -> Decoder )

logical_rx_frame_bytes = rx_media_bytes + one cam_hdr + one enc_hdr
logical_tx_frame_bytes = tx_media_bytes + one cam_hdr + one enc_hdr

network_tx_bytes = tx_media_bytes + tx_packets * ( "Eth" + "IPv4" + "UDP" + cam_hdr + enc_hdr )

integrity = 100 * tx_media_bytes / rx_media_bytes
```

For both validated routes:

```text
receive_s               = last_rx - first_arrival
internal_throughput_mbs = ( logical_rx_frame_bytes / 1,000,000 ) / receive_s
node_efficiency_pct     = 100 * active_process_ms / total_residency_ms
```

The `eth_errors`, `ipv4_errors`, `udp_errors`, & `nsh_errors` fields are snapshots of process-wide cumulative diagnostics, whereas Tx resubmission counters are frame / route local. This difference must be respected when aggregating CSV rows: summing the cumulative error columns would double-count historical failures.

---

## ⚙️ 9. Encoder — Hybrid Frame Processing, "CUDA", "Temporal" Control, & "H.265"

### 9.1 Role

The `Encoder` is intentionally "NSH"-unaware. It receives ordinary "UDP" packets whose application payload begins with the geometric context exposed by the `SFF2` proxy:

```text
[ geo_agg_hdr | cam_hdr | points ]
```

Its role combines two execution models.

At packet arrival it behaves in a data-plane-oriented manner: coordinates are decoded & written directly into deterministic frame offsets, packet-completion state is updated incrementally, & the most complete geometry snapshot is retained. During "CPU" / "GPU" work, the implementation continues servicing network input through cooperative polling.

At projection time it still requires a frame-level readiness condition because the exact active point set & final geometry must be known. The `Encoder` is therefore a **hybrid frame-aware element**, positioned between a conventional application that waits passively for a complete object & a pure packet-local data-plane function.

### 9.2 Frame Assembly

A frame buffer tracks:

```text
original_points
expected_packets
received_points
rx_packets
payload_bytes
packet_received bitmap
first_arrival
last_arrival
frame_ready
cam_hdr snapshot
geo_agg_hdr snapshot
```

Packet sequence numbers determine the destination offset. Point conversion therefore occurs while packets arrive rather than through a separate post-reception frame conversion pass.

A normal frame becomes processable only when all expected packets & points have arrived. At "EOS", a partial final frame can be compacted & processed if necessary, but the validated 300-frame run contains complete frames only.

### 9.3 Selecting the Most Complete Geometry Snapshot

`SFF1` emits progressive geometry. The `Encoder` consequently retains the snapshot with the greatest valid `active_point_count` observed for the frame.

For a complete frame, `geometry_from_sff1()` accepts the offloaded result only when:

```text
metadata_active_points == active_point_count
```

and all decoded geometry values are finite & internally valid.

This prevents an early progressive packet from being mistaken for the final geometric description.

### 9.4 Frame Readiness & Incomplete Frames

The normal processing path requires point completeness before projection because the six-view representation & "codec" input are frame-level products.

This barrier does not mean that all upstream work is deferred. Before readiness, the `Encoder` has already:

```text
validated packet headers
converted point coordinates
placed points at deterministic offsets
updated receive counters
tracked geometric metadata progression
recorded arrival timing
```

The remaining barrier is therefore associated with operations whose result is inherently frame-level rather than with passive waiting for every computation to begin.

### 9.5 Division of Geometric Work & Selectable Offload

The build provides:

```text
OFFLOAD_MODE_DISABLED = 0
OFFLOAD_MODE_ENABLED  = 1
OFFLOAD_MODE          = OFFLOAD_MODE_ENABLED   ( current run )
```

With offload enabled & valid complete "GAC" metadata, the `Encoder` obtains:

```text
centroid
extent
raw bounding-box centre
max_r
```

without recomputing the corresponding frame scans locally. For complete offloaded frames:

```text
geometry_aggregation_ms = 0
max_r_ms                 = 0
```

inside `Encoder` telemetry by design.

If metadata are unavailable, inconsistent, disabled, or incomplete, `geometry_recompute_local()` remains a functional fallback. It recomputes sums / extrema, derives centroid / extent / box centre, & performs the radius pass over the active point set while periodically polling "DPDK".

The final object scale used before projection is based on the radius target:

```text
target_radius = CAMERA_DISTANCE * 0.2
final_scale   = target_radius / max_r
```

subject to validity checks in the implementation.

This selectable path is important experimentally: it permits a controlled future `OFFLOAD_MODE_ENABLED` vs. `OFFLOAD_MODE_DISABLED` comparison without changing the packet-processing architecture.

### 9.6 Workload-Driven "Temporal" Controller

"Temporal" adaptation is located at the `Encoder` because the experimental bottleneck is the processing node rather than a user-side quality selector.

The controller observes a service-time sample `T_n` & applies an exponentially weighted moving average:

```text
T_base            = 1000 / TARGET_FPS
T_budget( skip )  = skip * T_base
E_n               = alpha * T_n + ( 1 - alpha ) * E_( n - 1 )
workload_ratio    = E_n / T_budget( active_skip )
```

Current parameters are:

```text
TARGET_FPS         = 30
EWMA_ALPHA         = 0.25
MAX_SKIP           = 9
MIN_FRAMES         = 3
STABLE_STREAK      = 3
MAX_FRAMES         = 15
OVERLOAD_STREAK    = 2
RECOVERY_STREAK    = 9
RETRY_FRAMES       = 3
OVERLOAD_RATIO     = 0.90
RECOVERY_RATIO     = 0.75
OVERLOAD_FRACTION  = 0.25
RECOVERY_FRACTION  = 0.10
```

The controller first waits for a stable start-up window. Once armed, overload is detected when at least one of the following is true:

```text
workload_ratio >= 0.90
wait_raw_queue_ms > 0.25 * active_budget_ms
frame_backlog >= 2
```

After two consecutive overload observations, the requested skip increases by one, up to `MAX_SKIP`.

Recovery is deliberately more conservative. For `active_skip > 1`, the controller evaluates whether the current "EWMA" would fit within `75 %` of the budget at `skip - 1`, while raw-queue wait stays below `10 %` of the current budget & frame backlog remains zero. Nine consecutive recoverable observations are required before reducing the skip.

If the `Camera` has not yet reflected a requested value, the `Encoder` retries the same control decision every three observations rather than continuously increasing the request.

Events are exported as:

```text
"WARMUP"
"IDLE"
"SKIP+1"
"SKIP-1"
"RETRY"
"INVALID"
```

The current validation run reports five start-up `WARMUP` records followed by 295 `IDLE` records. `current_skip` remains `1` for all 300 frames because the workload ratio remains well below the overload threshold & `frame_backlog` remains zero.

### 9.7 "CUDA" Memory Strategy

The projection pipeline preallocates persistent device buffers, a "CUDA" stream, & timing events. Host "I420" output slots are also allocated once & registered with "CUDA".

Current buffering parameters include:

```text
H2D_CHUNK_POINTS = 65536
YUV_BUFFER_COUNT = 3
```

The point array is transferred asynchronously in chunks. Before each chunk, the "CUDA" path can invoke the `Encoder`'s "DPDK" polling callback. After kernel launch & copy-back, the worker continues polling while `cudaStreamQuery()` reports the stream as incomplete.

This design limits the duration for which "GPU" submission prevents packet reception & avoids repeated device allocation on the measured path.

### 9.8 "CUDA" Projection Stages

The static pose of the current experiment allows the pipeline to avoid a separate point-wise transformed-bounding-box reduction.

The raw bounding-box centre is transformed analytically around the centroid & final scale:

```text
B'_x = ( B_x - C_x ) * final_scale
B'_y = ( B_y - C_y ) * final_scale
B'_z = ( B_z - C_z ) * final_scale + CAMERA_DISTANCE
```

The corresponding scaled extents determine a global orthographic scale. In implementation terms:

```text
global_scale = 1.10 * max( extent'_x / WIDTH, extent'_y / HEIGHT, extent'_z / WIDTH )
```

The `1.10` factor provides projection margin.

The subsequent stages are:

```text
asynchronous "H2D" transfer
fused point projection / colour conversion / z-buffer update
atlas packing
asynchronous "D2H" copy
```

Dynamic pose support may require revisiting this optimisation once pose-dependent transformations are introduced downstream.

### 9.9 Six-View G-Buffer

The fused "CUDA" kernel performs, per point:

```text
object-centred scaling
normalised-coordinate construction
"BT.601" "RGB" -> "YUV" conversion
six orthographic face projections
depth quantisation
atomic z-buffer visibility arbitration
geometry / occupancy / texture updates
```

`atomicMax()` resolves visibility conflicts in the integer depth buffer. This places the principal point-parallel work on the "GPU" while keeping only compact geometric control calculations on the host.

### 9.10 Atlas Geometry

The base projected view uses:

```text
WIDTH  = 640
HEIGHT = 480
```

Faces are padded to:

```text
FACE_W_PADDED = 640
FACE_H_PADDED = 512
```

and arranged in a four-by-three cross:

```text
CROSS_W = 2560
CROSS_H = 1536
```

Three vertically stacked crosses represent:

```text
"Geometry"
"Texture"
"Occupancy"
```

so the encoded "I420" frame is:

```text
ENCODER_W = 2560
ENCODER_H = 4608
```

The corresponding uncompressed "I420" buffer is:

```text
TOTAL_YUV_SIZE = 17,694,720 B
```

per submitted frame.

### 9.11 "GPU" Timing Probes

The `Encoder` exports:

```text
gpu_transfer_ms
gpu_kernel_ms
gpu_packing_ms
gpu_copyback_ms
host_overhead_ms
projection_ms
```

"CUDA" event timings isolate asynchronous "GPU" stages, whereas `projection_ms` is the complete host-visible interval around the projection call. Because cooperative "DPDK" polling can execute while the stream is incomplete, `host_overhead_ms` should be interpreted as residual host-visible time rather than as a pure arithmetic-"CPU" kernel metric.

### 9.12 Persistent "FFmpeg" / "NVENC" Process & Pre-Roll

A single persistent "FFmpeg" process is launched for the experiment. Current rate-control / latency-oriented settings include:

```text
"codec"            = hevc_nvenc
preset           = p2
tune             = ull
rate control     = cbr
target bitrate   = 10M
buffer size      = 20M
"GOP"              = 15
B frames         = 0
lookahead        = 0
delay            = 0
zero latency     = 1
flush packets    = 1
```

Before application frames are measured, the `Encoder` submits one "GOP" of private blank pre-roll frames:

```text
FRAMES = 15
```

The pre-roll uses the ordinary writer & parser path so the persistent "codec", muxer, & attribution logic enter a warmed operational state. Pre-roll frames are never transmitted as application network frames & do not populate the 300-row `Encoder` telemetry.

A dedicated writer thread & three registered "I420" slots decouple projection from blocking writes to "FFmpeg" stdin. If all slots are occupied, the `Encoder` keeps servicing network / "codec" activity while waiting for a reusable slot.

### 9.13 Why "MPEG-TS" / "PES" Parsing Is Necessary

Pipe reads from "FFmpeg" do not preserve video-frame boundaries. The `Encoder` therefore reconstructs fixed 188-byte "MPEG-TS" packets from arbitrary read sizes & identifies video "PES" starts.

The first associated video-"PES" boundary is matched to the oldest frame submitted to "FFmpeg". This enables:

```text
encode_h265_ms = first_associated_PES - ffmpeg_input_start
```

The metric includes asynchronous "codec", scheduling, muxing, & pipe-delivery effects. It is intentionally **not** presented as a pure "NVENC" hardware-kernel duration.

### 9.14 Encoder Output Packetisation

The `Encoder` groups seven complete TS packets:

```text
7 * 188 B = 1316 B
```

and prepends:

```text
cam_hdr
enc_hdr
```

before creating a plain "UDP" packet for the `SFF2` proxy.

No service-chain header is emitted by the `Encoder`. Primary-path "NSH" state remains an `SFF2` responsibility.

### 9.15 Encoder Telemetry — Complete Semantics

The `Encoder` exports **54 fields**, making it the most instrumented node in the implemented chain. The schema intentionally separates raw reception, local geometry, "CUDA" wall / device intervals, writer behaviour, workload control, "codec" emergence, compressed egress, & partial `Camera`-relative latency.

| Metric | Unit / Type | Exact Definition | Interpretation |
|---|---|---|---|
| `frame_id` | identifier | Original Camera frame identifier. | Binds raw input, geometry, controller state, "codec" attribution & compressed output. |
| `status` | boolean | `1` for a valid frame record; set to `0` on invalid geometry / enqueue conditions. | Functional validity flag. |
| `current_skip` | factor | "Temporal" factor carried by the Camera frame that generated this Encoder record. | The workload controller may request a different value for subsequent source frames. |
| `event` | state | One of "WARMUP", "IDLE", "SKIP+1", "SKIP-1", "RETRY", or "INVALID". | Makes controller state transitions explicit. |
| `yaw` | degrees / reference | Current upstream yaw value; `0.0` in the validated run. | Static until the future "Pose" path is connected to Decoder-side reconstruction. |
| `pitch` | degrees / reference | Current upstream pitch value; `0.0` in the validated run. | Static reference. |
| `zoom` | factor | Current upstream zoom value; `1.0` in the validated run. | Static reference. |
| `camera_send_timestamp` | seconds | Camera frame Tx-start timestamp propagated in `cam_hdr`. | Source timing anchor. |
| `recv_start_timestamp` | seconds | First Encoder packet-arrival time for the frame. | Encoder node-entry anchor. |
| `node_exit_timestamp` | seconds | Last compressed "DPDK" egress timestamp attributed to the frame. | Current Encoder node-exit anchor. |
| `clock_offset_ms` | ms | For the first processed frame: `( first_encoder_arrival - first_camera_tx ) * 1000`; reused as a baseline. | Removes the initial shared-host path offset from Encoder Camera-relative latency fields. |
| `original_points` | points | Point count declared by the Camera. | Input completeness denominator. |
| `rx_points` | points | Unique point records reassembled by packet sequence. | Compared against `original_points`. |
| `tx_points` | points represented | Set equal to the point population represented by the encoded frame. | Does **not** mean that raw point records are transmitted on Route 1. |
| `rx_packets` | packets | Validated point packets contributing to the frame. | Input segmentation. |
| `tx_packets` | packets | Compressed-media "UDP" packets accepted by the Encoder Tx path. | Output segmentation. |
| `payload_bytes` | bytes | Sum of input point bytes only. | Excludes incoming `geo_agg_hdr` / `cam_hdr`. |
| `data_integrity_pct` | % | `100 * rx_points / original_points`. | Raw point-population integrity before projection. |
| `internal_throughput_mbs` | MB/s | `( payload_bytes + one cam_hdr ) / 1e6 / ( last_arrival - first_arrival )`. | Logical raw-input throughput over the pure receive span. |
| `logical_bitrate_mbps` | Mbit/s | `( mpeg_bytes_generated + one cam_hdr + one enc_hdr ) * 8 * effective_fps / 1e6`. | Compressed logical-output bitrate; intentionally not the raw-input bitrate used by the reference application. |
| `network_bitrate_mbps` | Mbit/s | `( mpeg_bytes_generated + tx_packets * ( "Eth" + IPv4 + "UDP" + cam_hdr + enc_hdr ) ) * 8 * effective_fps / 1e6`. | Compressed output including repeated packet headers. |
| `conversion_ms` | ms | Accumulated per-packet network-to-host coordinate conversion & placement into the frame buffer. | Occurs incrementally as packets arrive. |
| `geometry_aggregation_ms` | ms | Local centroid / extrema / bounding-box aggregation time when the `SFF1` offload cannot be used. | Exactly `0` for complete offloaded frames in the representative run. |
| `max_r_ms` | ms | Local exact farthest-point radius pass when required. | Exactly `0` for complete offloaded frames; partial "EOS" recovery may still recompute it. |
| `projection_ms` | ms | Wall-clock interval from projection submission start to completion, including cooperative "DPDK" polling while asynchronous "CUDA" work progresses. | Broader than the sum of pure device event times. |
| `tx_duration_ms` | ms | Wall-clock interval of the dedicated writer while submitting one complete "I420" frame to the persistent "FFmpeg" stdin. | Includes blocking pipe behaviour; it is the reference-compatible render / "codec" handoff component. |
| `active_process_ms` | ms | Current implementation: `conversion + geometry_aggregation + max_r + projection + tx_duration`. | Kept equal to `total_processing_ms` for reference-compatible node-efficiency semantics. |
| `total_processing_ms` | ms | Same arithmetic sum as `active_process_ms` in the current snapshot. | Does not include asynchronous `encode_h265_ms` a second time. |
| `total_residency_ms` | ms | `last_encoded_dpdk_egress - first_point_arrival`. | Includes frame assembly, queueing, projection, writer / "codec" exposure & compressed egress. |
| `node_efficiency_pct` | % | `100 * active_process_ms / total_residency_ms`. | Processing fraction of the broader Encoder residence. |
| `gpu_transfer_ms` | ms | "CUDA" event interval from projection start through asynchronous "H2D" point transfer. | Device-timeline transfer component. |
| `gpu_kernel_ms` | ms | "CUDA" event interval from end of "H2D" transfer through fused 6-view "G-Buffer" projection. | Includes object-centric transform, "BT.601" conversion, projection & atomic depth selection. |
| `gpu_packing_ms` | ms | "CUDA" event interval for atlas / "I420" packing after the projection kernel. | Device packing component. |
| `gpu_copyback_ms` | ms | "CUDA" event interval for the final device-to-host "I420" copy. | Device-timeline "D2H" component. |
| `host_overhead_ms` | ms | `max( 0, projection_ms - gpu_transfer_ms - gpu_kernel_ms - gpu_packing_ms - gpu_copyback_ms )`. | Residual wall time, including host orchestration & cooperative polling; **not** a pure "CPU"-compute probe. |
| `camera_to_node_latency_ms` | ms | `first_arrival - camera_tx - clock_offset`. | Baseline-corrected Camera-to-Encoder-arrival variation; the first frame is intentionally near `0`. |
| `end_to_end_latency_ms` | ms | `node_exit - camera_send_timestamp - clock_offset`. | Partial Camera-to-Encoder-compressed-egress latency, **not** final Camera-to-User E2E. |
| `schedule_delay_ms` | ms | `( service_start - first_encoder_session_arrival ) - frame_offset / TARGET_FPS`. | Measures service-start deviation from the ideal source schedule. |
| `network_jitter_ms` | ms | Absolute first-arrival spacing error relative to `( frame_id_gap / TARGET_FPS )`. | ID-gap-aware timing dispersion. |
| `wait_raw_queue_ms` | ms | `service_start - frame_ready`; if the full frame is not complete at finalisation, the fallback reference is `last_arrival`. | Raw-frame waiting after readiness & before Encoder service begins. |
| `wait_render_queue_ms` | ms | `ffmpeg_write_start - projection_end`. | Post-projection writer-queue wait only; pre-projection "I420" slot acquisition is deliberately excluded. |
| `workload_ewma_ms` | ms | `E_n = alpha * service_n + ( 1 - alpha ) * E_(n-1)` with `alpha = 0.25`; reset to the current sample when warm-up becomes armed. | Smoothed Encoder worker-service signal used by temporal adaptation. |
| `workload_ratio` | ratio | `workload_ewma_ms / ( current_skip * 1000 / TARGET_FPS )`. | Controller load relative to the active temporal budget. |
| `frame_backlog` | frames | Number of other frame buffers pending after selecting the current frame (`frame_buffers.size() - 1`). | Direct raw-input backlog trigger; overload is true when this reaches at least `2`. |
| `codec_backlog` | frames | `max( writer_pending_frames(), mpeg_frame_queue.size() )`. | Diagnostic "codec" / writer pressure. In the present controller implementation it is **observed but not directly used** in the overload / recovery boolean. |
| `encode_h265_ms` | ms | Elapsed time from "FFmpeg" writer start for a real frame to detection of the first attributed video-"PES" boundary. | First observable encoded-output latency, not a pure "NVENC" kernel execution time. |
| `mpeg_bytes_generated` | bytes | "MPEG-TS" bytes attributed by the parser to the source frame before network-packet allocation. | With zero `mbuf_starvation`, this equals the media-byte population received by `SFF2` Route 1 in the representative run. |
| `ffmpeg_write_calls` | count | Number of `write()` system calls used to submit the frame to "FFmpeg" stdin. | Current run records one call per real frame. |
| `ffmpeg_write_eagain` | count | Number of defensive `EAGAIN` / `EWOULDBLOCK` results encountered by the writer. | Expected to remain zero with the current blocking input pipe. |
| `tx_zero_accepts` | count | Compressed-output Tx calls returning zero accepted packets. | Local "DPDK" backpressure. |
| `tx_partial_accepts` | count | Compressed-output Tx calls with partial acceptance. | Local "DPDK" backpressure. |
| `tx_resubmit_calls` | count | Compressed-output resubmission calls after incomplete acceptance. | Not a "UDP" retransmission count. |
| `tx_resubmitted_packets` | packet-attempts | Packet requests made by those resubmission calls. | May recount a pending packet. |
| `mbuf_starvation` | count | Compressed-output `mbuf` allocation / append failures. | If non-zero, `mpeg_bytes_generated` can exceed bytes actually transmitted. |

The current processing & residency equations are:

```text
total_processing_ms = conversion_ms + geometry_aggregation_ms + max_r_ms + projection_ms + tx_duration_ms

active_process_ms = total_processing_ms

node_efficiency_pct = 100 * active_process_ms / total_residency_ms

total_residency_ms = last_compressed_dpdk_egress - first_point_arrival
```

When `OFFLOAD_MODE_ENABLED` receives a complete final `geo_agg_hdr` whose `active_point_count` matches the assembled point population:

```text
geometry_aggregation_ms = 0
max_r_ms                = 0
```

because the `Encoder` consumes the centroid, extents, bounding-box centre, & final radius produced upstream. With offload disabled, or if metadata cannot be accepted, the same geometry is recomputed locally. This is the explicit control required to evaluate the computational-placement benefit rather than merely assuming it.

The workload controller uses:

```text
T_base          = 1000 / TARGET_FPS
T_budget( skip ) = skip * T_base
E_n             = EWMA_ALPHA * service_n + ( 1 - EWMA_ALPHA ) * E_( n - 1 )
workload_ratio  = E_n / T_budget( skip )
```

with the current constants:

```text
EWMA_ALPHA       = 0.25
OVERLOAD_RATIO   = 0.90
RECOVERY_RATIO   = 0.75
OVERLOAD_FRACTION = 0.25
RECOVERY_FRACTION = 0.10
OVERLOAD_STREAK  = 2
RECOVERY_STREAK  = 9
MAX_SKIP         = 9
```

The implemented overload predicate is:

```text
overloaded = workload_ratio >= 0.90 OR wait_raw_queue_ms > 0.25 * active_budget_ms OR frame_backlog >= 2
```

`codec_backlog` is exported as an important diagnostic but does **not** currently enter this boolean directly. This distinction is relevant when interpreting future stress experiments.

Finally, `encode_h265_ms` must remain independent from `total_processing_ms`: the former reaches the first detected video-"PES" output after the frame enters the persistent "FFmpeg" writer, whereas `total_processing_ms` already counts the "I420" writer interval required by the original node-processing boundary. Adding the two would mix overlapping asynchronous stages.

### 9.16 Encoder "End-of-Stream" Handling

When "EOS" arrives, the `Encoder` finalises any eligible residual frame, drains outstanding writer jobs & encoded output, closes "FFmpeg" input, completes the parser drain, flushes pending "DPDK" output, & only then writes the final telemetry.

This ordering is necessary because encoded bytes can remain in the "codec" / muxer pipeline after the last application frame has been submitted. The final frame can consequently display a larger residency or compressed-media delay than steady-state frames; it should be analysed as a tail-drain condition rather than silently discarded.

---

## 🖥️ 10. "CPU", "GPU", & Core-Constrained Execution

The launcher exposes an eight-logical-"CPU" execution model ( `0-7` ). Core placement is part of the experiment & must be preserved when results are compared.

| Logical "CPU" | Assignment in Current Launcher | Rationale |
|---:|---|---|
| `0` | "OVS" auxiliary "lcore" + Encoder "FFmpeg" affinity; also included in the future Decoder cpuset | Housekeeping / "codec" compromise on the constrained host |
| `1` | "OVS"-"DPDK" "PMD" | Dedicated virtual-switch data-path processing |
| `2` | Camera | Dedicated Camera "DPDK" worker |
| `3` | `SFF1`; future `SFF3` container uses the same cpuset | Only `SFF1` is active in the validated upstream hot path |
| `4` | `SFF2` | Dedicated four-port forwarder / proxy |
| `5` | Encoder | Principal Encoder "DPDK" / C++ worker |
| `6` | Decoder | Reserved for future Decoder processing; Decoder container also exposes Core `0` |
| `7` | User | Reserved for rendering / client logic |

The current "OVS" configuration is:

```text
dpdk-"lcore"-mask = 0x1   -> Core 0
pmd-cpu-mask    = 0x2   -> Core 1
dpdk-socket-mem = 1024 MiB
```

The host launcher allocates:

```text
2048 * 2 MiB HugePages = 4096 MiB ~= 4 GiB
```

### Core 0 Sharing

Running "FFmpeg" on Core `0` is a deliberate compromise. It prevents the "codec" writer / child process from consuming the dedicated `Encoder` "DPDK" core, but it can contend with "OVS" auxiliary activity & host housekeeping.

The current telemetry shows that this arrangement is sufficient for the validated upstream run, but it must be re-evaluated when the `Decoder` becomes active because its container also includes Core `0` & Core `6`.

### Why the Current Constraint Is Manageable

The implementation reduces pressure on the limited "CPU" set through:

```text
dedicated "OVS"-"DPDK" "PMD" placement
native-node "CPU" affinity
optional "Linux" "CPU" isolation
"GAC" geometry offload
static-pose "CUDA" optimisation
persistent "CUDA" buffers / events
"GPU" projection
"NVENC" compression
persistent pre-rolled "FFmpeg" process
triple "I420" buffering
cooperative "DPDK" polling during "CPU" / "GPU" waits
workload-driven source temporal regulation
```

The final complete chain will contain more simultaneous work than the current upstream subset. "CPU" placement therefore remains part of the experimental methodology rather than a permanently solved scaling problem.

---

## 11. "HugePages" & Optional "CPU" Isolation

The experiment configures "DPDK" memory explicitly before topology creation.

The current `init_all.sh` allocates:

```text
nr_hugepages = 2048
HugePage size = 2 MiB
total         = 4 GiB
```

It also drops filesystem caches before the environment is started. `WARM_MODE_ENABLED` is subsequently applied by the `Camera` itself, so host cache-reset policy & `Camera` warm-mode policy should both be recorded when reproducing a run.

The optional isolation scripts configure:

```text
isolcpus=1-7
```

leaving Core `0` as the principal non-isolated housekeeping "CPU".

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

The ordinary `init_all.sh` / `stop_all.sh` lifecycle does not toggle the reboot-level `isolcpus` setting. "CPU" isolation, "Docker" cpusets, "OVS" "PMD" placement, & "codec" affinity must therefore be reported as separate experimental conditions.

---

## 12. Container & "OVS"-"DPDK" Environment

The repository documentation retains the common build environment used by the project:

```text
nvidia/cuda:12.2.0-devel-ubuntu22.04
"DPDK" 22.11.4
```

The image also provides GCC / G++, Meson, Ninja, NUMA development support, "FFmpeg", `tcpdump`, `ethtool`, & the utilities required by the current launch scripts. These version statements should be archived together with each benchmark if the base image changes in a later revision.

Native service functions are launched with:

```text
--net none
--privileged
```

The nodes communicate through explicitly mounted "DPDK" "vhost-user" sockets rather than "Docker"'s ordinary network stack.

Each service container receives the relevant mounts for:

```text
/dev/hugepages
/tmp
/shared
/app
```

and an explicit `DPDK_CORE` value. The project source directory is mounted as `/app`, so node entrypoints execute the current repository source rather than a stale source tree embedded in a container image.

The launcher starts implemented nodes sequentially & waits for their required "vhost-user" sockets before advancing. This reduces topology-startup races & makes the active data-path bindings explicit.

### Build-Time Specialisation

The `Encoder` build remains intentionally performance-oriented. The compilation profile documented by the repository snapshot is:

```text
C++   : -O3 -march=native -ffast-math -funroll-loops -std=c++14
"CUDA": -O3 -arch=sm_61 -std=c++14
```

These flags are part of the experimental condition. `-march=native` specialises host object code for the build machine, `-ffast-math` permits non-conservative floating-point transformations, & `-arch=sm_61` fixes the generated device target. A benchmark rebuilt with another processor, compiler, or "GPU" architecture is therefore not automatically performance- or bit-equivalent.

Any benchmark intended for comparison should therefore retain:

```text
compiler versions
host optimisation flags
"CUDA" architecture target
"GPU" model / driver
"FFmpeg" / "NVENC" version
"DPDK" / "OVS" versions
```

### "OVS"-"DPDK" Topology

The virtual switch creates one `netdev` bridge:

```text
br-sfc
```

and explicit adjacent "vhost-user" links:

```text
Camera <-> SFF1 <-> SFF2 <-> Encoder
                     |  |
                     |  +-> Decoder
                     +----> SFF3 <-> User
```

"OpenFlow" rules only connect declared adjacent endpoints. A final default-deny rule drops unmatched traffic.

This separation is important: "OVS"-"DPDK" provides deterministic connectivity, while `SFF1` & `SFF2` implement application-aware service computation, service-path classification, & proxy semantics.

The topology script can optionally enable internal mirror ports for `tcpdump` inspection. Debug mirroring changes the environment & should remain disabled in benchmark runs unless packet capture overhead is itself being studied.

---

## 13. Repository Structure

The current repository organisation is conceptually:

```text
Thesis/
├── README.md
├── docs/                                  # Thesis / reference material
├── env/                                   # "Python" virtual environment
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

---

## 🧬 14. Dataset, "Python" Environment, & Offline Preparation

### 14.1 Research Dataset — 8i Voxelized Full Bodies ( "Loot" )

The experimental point-cloud source is the **"Loot" sequence from the 8i Voxelized Full Bodies ( 8iVFB v2 ) dataset**, provided by 8i Labs & publicly documented through the JPEG Pleno database.

The original dataset defines four dynamic full-body sequences:

```text
longdress
loot
redandblack
soldier
```

Each sequence contains a full human subject captured by 42 "RGB" cameras organised in 14 clusters, at 30 frames/s for approximately 10 s. The depth-10 representation uses a `1024 x 1024 x 1024` voxel grid, with "RGB" colour attributes associated with occupied voxels.

The current experiment uses the complete 300-frame depth-10 "Loot" sequence:

```text
loot_vox10_1000.ply
...
loot_vox10_1299.ply
```

The prepared "Loot" dataset snapshot documented by this repository reports:

| Dataset quantity | Current "Loot" snapshot |
|---|---:|
| Frames | `300` |
| Total points | `238,146,391` |
| Mean points / frame | `793,821` |
| Minimum points / frame | `770,822` |
| Maximum points / frame | `835,458` |
| Aggregate source PLY footprint | `5,144,378,340 B` ( `5.144 GB` ) |
| Aggregate generated BIN footprint | `3,810,342,256 B` ( `3.810 GB` ) |
| BIN footprint reduction vs. source PLY | `25.93 %` |
| Mean offline conversion time | `6.310 s / frame` |

The dataset is available online from the JPEG Pleno database under the accompanying 8i license terms. The requested academic citation is:

> E. d'Eon, B. Harrison, T. Myers, & P. A. Chou, *8i Voxelized Full Bodies — A Voxelized Point Cloud Dataset*, ISO/IEC JTC1/SC29 Joint WG11/WG1 input document WG11M40059/WG1M74006, Geneva, January 2017.

Repository users should consult the dataset page & license directly before use or redistribution:

```text
https://plenodb.jpeg.org/pc/8ilabs/
```

### 14.2 Repository Data Policy

Neither the original `.ply` frames nor the generated `.bin` frames are committed to this repository.

This is intentional: the complete "Loot" PLY sequence is approximately `5.14 GB` in the current local representation, & the converted binary sequence is still approximately `3.81 GB`. Keeping both variants outside Git avoids an unnecessarily large repository & prevents ordinary source-control operations from becoming dominated by experimental data.

The repository therefore contains the **code, data layout, conversion procedure, & telemetry**, while the large dataset artefacts are expected to be obtained / generated locally.

This size-based repository policy is separate from the dataset licence. Any local copy or redistribution of 8i material must also respect the licence distributed with the dataset.

### 14.3 Binary Representation Used by the Camera

The offline converter transforms each PLY frame into a contiguous header-less sequence matching `point_tx`:

```text
float32 x
float32 y
float32 z
uint8   r
uint8   g
uint8   b
uint8   padding
```

Thus:

```text
bytes_per_point = 16
```

The transformation removes PLY parsing & per-field conversion from the `Camera`'s streaming hot path. It is a **storage / parsing preparation step**, not a compression algorithm in the information-theoretic or rate-distortion sense.

The current scale factor documented by the repository is:

```text
SCALE_FACTOR = 1.0
```

### 14.4 "Python" Environment

The root-level:

```text
env/
```

is the "Python" environment used by the current offline utilities, primarily the point-cloud converter.

Activate it from the repository root with:

```bash
source env/bin/activate
```

The README snapshot documents the converter dependencies as:

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

`pandas` & `matplotlib` are not required merely to execute the documented converter. They are appropriate for higher-level telemetry analysis but are not part of the native "DPDK" hot path.

### 14.5 Offline Converter

Typical execution is:

```bash
source env/bin/activate
python3 src/shared/py/converter/converter.py
deactivate
```

The converter is an **offline preparation stage**. Its elapsed time, including PLY parsing & BIN writing, must not be merged with `Camera`, SFF, `Encoder`, "CUDA", or "codec" latency measurements.

The converter telemetry is nevertheless useful for reproducibility because it records the exact frame population & the source / generated data footprint used by the experiment. Its complete source-defined schema is:

| Metric | Unit / Type | Exact Meaning |
|---|---|---|
| `filename` | string | Source `.ply` file processed by the offline stage. |
| `status` | string | `success` when the complete conversion path terminates without exception; otherwise `error`. |
| `num_points` | points | Number of vertices read from the PLY `vertex` element & written to the fixed-width binary representation. |
| `read_ascii_ms` | ms | Wall-clock interval spent inside `PlyData.read( file_path )`. |
| `write_bin_ms` | ms | Wall-clock interval spent writing the already constructed contiguous `network_array` to the `.bin` file. |
| `conversion_ms` | ms | Whole-file interval from the beginning of file processing through parsing, coordinate / colour extraction, fixed-record construction, & output handling. |
| `size_ascii_bytes` | bytes | Size of the source `.ply` artefact obtained from `os.path.getsize()`. |
| `size_bin_bytes` | bytes | Size of the generated fixed-width `.bin` artefact obtained from `os.path.getsize()`. |

`conversion_ms` is intentionally broader than `read_ascii_ms + write_bin_ms`. The difference includes the point-array extraction, `SCALE_FACTOR` application, numeric casting, explicit padding construction, & population of the 16-byte structured representation. Consequently, it must **not** be reconstructed as the sum of the two narrower I / O probes.

The streaming results in Section 18 start from the prepared binary sequence. Offline Converter time is therefore excluded from `Camera` / `SFF1` / `SFF2` / `Encoder` latency & should be reported only together with the matching Converter telemetry of the dataset-preparation run.

---

## 🚀 15. Running the Experiment

### 15.1 Obtain & Prepare the Dataset

1. Obtain the 8iVFB v2 dataset from the official / JPEG Pleno distribution & retain its licence information.
2. Place the local "Loot" PLY frames in the repository's expected data directory.
3. Run the offline converter from the repository root:

```bash
source env/bin/activate
python3 src/shared/py/converter/converter.py
deactivate
```

The `.ply` & generated `.bin` datasets are intentionally not versioned in Git because of their multi-gigabyte size.

### 15.2 Optional: Enable "CPU" Isolation

From `src/`:

```bash
sudo ./enable_isolcpus.sh
sudo reboot
```

After reboot, return to the repository & verify `/proc/cmdline` before benchmarking.

### 15.3 Start the Environment

From `src/`:

```bash
sudo ./init_all.sh
```

The launcher is responsible for the host-side "DPDK" preparation & for invoking the topology / container startup sequence. When `init_all.sh` is used, the infrastructure scripts should not be launched a second time unless the user intentionally tears down & reconstructs the topology.

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

"CPU" isolation is a reboot-level setting & remains independent of the ordinary container / "OVS" shutdown path.

---

## 16. Entrypoint Execution Model

Each implemented native node is launched by an `entrypoint.sh` inside its container.

The entrypoint typically:

1. compiles the current mounted source with optimisation enabled;
2. links against the "DPDK" libraries available in the common image;
3. launches the "EAL" on the logical core passed through `DPDK_CORE`;
4. creates one or more `virtio_user` devices;
5. binds them to the corresponding `/tmp/vh-*` "vhost-user" sockets;
6. starts the node's run-to-completion loop.

For example, the `Camera` is compiled with an optimised GCC invocation linked through `pkg-config` & is launched directly against its "virtio-user" "vhost" socket.

This runtime compilation approach is useful during thesis development because the mounted source remains the single authoritative implementation.

---

## 📊 17. Native Telemetry Files

The current snapshot exports native telemetry under:

```text
src/shared/log/
```

Principal files include:

| Component | Telemetry |
|---|---|
| Converter | `log/converter/telemetry_converter.csv` |
| Camera | `log/camera/telemetry_camera.csv` |
| SFF1 / "GAC" | `log/sff1/telemetry_sff1.csv` |
| SFF2 Route 0 | `log/sff2/telemetry_sff1_enc.csv` |
| SFF2 Route 1 | `log/sff2/telemetry_enc_dec.csv` |
| SFF2 Route 2 | `log/sff2/telemetry_dec_sff3.csv` ( path reserved; quantitative semantics deferred ) |
| Encoder | `log/encoder/telemetry_encoder.csv` |
| "FFmpeg" | `log/encoder/ffmpeg.txt` |

The archived validation data contain 300 frame rows for `Camera`, `SFF1`, `SFF2` Route 0, `Encoder`, & `SFF2` Route 1. Their current schemas contain respectively **23**, **36**, **36**, **54**, & **36** columns. Those files are the authoritative quantitative basis for Section 18, while the `ffmpeg.txt` stream provides the independent "codec"-side `vstats` cross-check.

### 17.1 Exact CSV Schema Reference

The following lists reproduce the headers of the validated CSV files exactly. They are intentionally verbose: the README is meant to function as a measurement guide, so a field must not need to be inferred from source code before a trace can be interpreted. Detailed formulae & boundaries are given in Sections 6.6, 7.6, 8.7, & 9.15.

**Camera — 23 fields**

```text
frame_id,
status,
current_skip,
last_control_frame,
timestamp_start_tx,
tx_points,
tx_packets,
payload_bytes,
internal_throughput_mbs,
logical_bitrate_mbps,
network_bitrate_mbps,
disk_io_ms,
serialization_ms,
tx_duration_ms,
active_tx_ms,
active_process_ms,
total_residency_ms,
node_efficiency_pct,
tx_zero_accepts,
tx_partial_accepts,
tx_resubmit_calls,
tx_resubmitted_packets,
mbuf_starvation
```

**SFF1 / "GAC" — 36 fields**

```text
frame_id,
status,
current_skip,
camera_send_timestamp,
recv_start_timestamp,
node_exit_timestamp,
original_points,
rx_points,
tx_points,
rx_packets,
tx_packets,
payload_bytes,
data_integrity_pct,
internal_throughput_mbs,
logical_bitrate_mbps,
network_bitrate_mbps,
tx_duration_ms,
active_tx_ms,
active_process_ms,
geometry_aggregation_ms,
max_r_ms,
cycle_ms,
header_wait_ms,
total_residency_ms,
node_efficiency_pct,
camera_to_node_latency_ms,
schedule_delay_ms,
network_jitter_ms,
eth_errors,
ipv4_errors,
udp_errors,
nsh_errors,
tx_zero_accepts,
tx_partial_accepts,
tx_resubmit_calls,
tx_resubmitted_packets
```

**SFF2 Route 0 — 36 fields**

```text
frame_id,
status,
current_skip,
camera_send_timestamp,
recv_start_timestamp,
node_exit_timestamp,
original_points,
rx_points,
tx_points,
rx_media_bytes,
tx_media_bytes,
rx_packets,
tx_packets,
payload_bytes,
data_integrity_pct,
internal_throughput_mbs,
logical_bitrate_mbps,
network_bitrate_mbps,
tx_duration_ms,
active_tx_ms,
active_process_ms,
cycle_ms,
header_wait_ms,
total_residency_ms,
node_efficiency_pct,
camera_to_node_latency_ms,
schedule_delay_ms,
network_jitter_ms,
eth_errors,
ipv4_errors,
udp_errors,
nsh_errors,
tx_zero_accepts,
tx_partial_accepts,
tx_resubmit_calls,
tx_resubmitted_packets
```

**Encoder — 54 fields**

```text
frame_id,
status,
current_skip,
event,
yaw,
pitch,
zoom,
camera_send_timestamp,
recv_start_timestamp,
node_exit_timestamp,
clock_offset_ms,
original_points,
rx_points,
tx_points,
rx_packets,
tx_packets,
payload_bytes,
data_integrity_pct,
internal_throughput_mbs,
logical_bitrate_mbps,
network_bitrate_mbps,
conversion_ms,
geometry_aggregation_ms,
max_r_ms,
projection_ms,
tx_duration_ms,
active_process_ms,
total_processing_ms,
total_residency_ms,
node_efficiency_pct,
gpu_transfer_ms,
gpu_kernel_ms,
gpu_packing_ms,
gpu_copyback_ms,
host_overhead_ms,
camera_to_node_latency_ms,
end_to_end_latency_ms,
schedule_delay_ms,
network_jitter_ms,
wait_raw_queue_ms,
wait_render_queue_ms,
workload_ewma_ms,
workload_ratio,
frame_backlog,
codec_backlog,
encode_h265_ms,
mpeg_bytes_generated,
ffmpeg_write_calls,
ffmpeg_write_eagain,
tx_zero_accepts,
tx_partial_accepts,
tx_resubmit_calls,
tx_resubmitted_packets,
mbuf_starvation
```

**SFF2 Route 1 — 36 fields**

```text
frame_id,
status,
current_skip,
camera_send_timestamp,
recv_start_timestamp,
node_exit_timestamp,
original_points,
rx_points,
tx_points,
rx_media_bytes,
tx_media_bytes,
rx_packets,
tx_packets,
payload_bytes,
data_integrity_pct,
internal_throughput_mbs,
logical_bitrate_mbps,
network_bitrate_mbps,
tx_duration_ms,
active_tx_ms,
active_process_ms,
cycle_ms,
header_wait_ms,
total_residency_ms,
node_efficiency_pct,
camera_to_node_latency_ms,
schedule_delay_ms,
network_jitter_ms,
eth_errors,
ipv4_errors,
udp_errors,
nsh_errors,
tx_zero_accepts,
tx_partial_accepts,
tx_resubmit_calls,
tx_resubmitted_packets
```

`SFF2` Route 2 retains a reserved output path, but its application-body contract & route-specific finalisation semantics are not stable. For that reason, the existence of `telemetry_dec_sff3.csv` is **not** treated as evidence of a validated 36-field Route-2 measurement stream.

### 17.2 "FFmpeg" `vstats` Field Semantics

The persistent "FFmpeg" process writes an independent `vstats` record for every coded input frame, including the private pre-roll. The fields visible in `ffmpeg.txt` are interpreted as follows:

| Field | Unit / Type | Meaning |
|---|---|---|
| `out` | index | "FFmpeg" output-file index associated with the statistic. The current command has a single "MPEG-TS" output, therefore the value is `0`. |
| `st` | index | Output stream index. The present video-only path reports stream `0`. |
| `frame` | count | Sequential encoded-frame number in the persistent "FFmpeg" process, therefore including private pre-roll frames. |
| `q` | encoder-reported quantiser indicator | Quantisation value reported by "FFmpeg" for the encoded frame. Under "NVENC" CBR control it is diagnostic rather than a user-selected constant-QP setting. |
| `f_size` | bytes | Encoded video-frame size reported by `vstats`, before the repository's frame-attributed "MPEG-TS" byte accounting. |
| `s_size` | bytes / displayed size | Cumulative coded size reported by "FFmpeg" up to the current frame. |
| `time` | seconds | Encoded media timeline associated with the current frame. |
| `br` | kbit/s | Frame-local bitrate statistic reported by "FFmpeg". |
| `avg_br` | kbit/s | Cumulative average bitrate statistic reported by "FFmpeg" up to the current record. |
| `type` | frame type | Coded picture class ( e.g., `I`, `P`, or `B` ). The present configuration disables B-frames. |

The 15 pre-roll records must be retained when analysing "codec" warm-up, but excluded when counting application frames. Conversely, `mpeg_bytes_generated` is the `Encoder`-side "MPEG-TS" byte quantity attributed to a real source frame, so it is expected to exceed `f_size` because transport-stream / muxing overhead is included.

### 17.3 Timing Quantities Must Not Be Added Indiscriminately

The telemetry deliberately distinguishes wall-clock residence, active work, asynchronous "codec" output, "GPU" event intervals, & local Tx-ring activity.

Common node residence is interpreted as:

```text
residency = node_exit - node_entry
```

but the exact entry / exit event depends on the function. In particular, current `Encoder` residency extends from first point arrival to the last encoded "DPDK" egress attributed to that frame.

Current active-process definitions are:

```text
Camera active_process_ms = disk_io_ms + serialization_ms + tx_duration_ms

SFF1 active_process_ms   = accumulated packet / geometry / Tx work
SFF2 active_process_ms   = accumulated route processing / Tx work

Encoder active_process_ms = conversion_ms + geometry_aggregation_ms + max_r_ms + projection_ms + tx_duration_ms
```

For all implemented nodes:

```text
node_efficiency_pct = 100 * active_process_ms / total_residency_ms
```

subject to the node-specific residence boundary above.

Throughput also has a precise semantic domain. `Camera` throughput is calculated over its frame send interval. `SFF1` / `SFF2` / `Encoder` input throughput uses the first-to-last receive span for the corresponding logical frame payload. `logical_bitrate_mbps` excludes repeated transport headers, whereas `network_bitrate_mbps` includes the repeated output envelope relevant to that node.

Several values must remain separate:

```text
active_tx_ms        -> time inside local rte_eth_tx_burst() calls
tx_duration_ms      -> complete wall-clock transmission / writer interval
encode_h265_ms      -> "FFmpeg" input start to first attributed video "PES"
"GPU" event metrics   -> asynchronous device-stage intervals
```

For the `Encoder`:

```text
total_processing_ms = active_process_ms
```

in the current implementation, but:

```text
encode_h265_ms
```

is asynchronous & must **not** be added to the processing sum.

Likewise, `wait_render_queue_ms` measures only the post-projection interval before the writer begins, while slot acquisition & raw-frame queue behaviour are represented elsewhere.

Finally, `Encoder` `end_to_end_latency_ms` currently ends at `Encoder` compressed-media egress. It is a partial `Camera`-to-`Encoder`-output metric & must not be labelled as final `Camera`-to-`User` latency.

---

## 🧪 18. Representative Results from the Current Validated Snapshot

The representative telemetry contains **300 frame records** for `Camera`, `SFF1`, `SFF2` Route 0, `Encoder`, & `SFF2` Route 1. The associated "FFmpeg" log contains the 15 private pre-roll frames plus the same 300 application frames.

The validated upstream path is:

```text
prepared 8i "Loot" BIN
-> Camera
-> SFF1 ( "GAC" )
-> SFF2 ( Route 0 / proxy )
-> Encoder
-> SFF2 ( Route 1 )
```

The measurements below describe the exact snapshot represented by the source configuration stored in this snapshot:

```text
K_FRAMES                  = 300
TARGET_FPS                = 30
Camera CACHE_MODE         = CACHE_MODE_MIDDLE
Camera WARM_MODE          = WARM_MODE_ENABLED
POINTS_PER_PACKET         = 80
Encoder OFFLOAD_MODE      = OFFLOAD_MODE_ENABLED
TEMPORAL_ADAPTATION       = TEMPORAL_ADAPTATION_ENABLED
H2D_CHUNK_POINTS          = 65536
YUV_BUFFER_COUNT          = 3
"NVENC" target bitrate    = 10M
"NVENC" buffer size       = 20M
"GOP"                     = 15
pre-roll                  = 15 frames
```

All 300 frame records are successful in every validated native node. No frame-integrity loss is observed across the implemented upstream path.

### 18.1 Dataset & Streaming Population

The current `Camera` / SFF / `Encoder` telemetry independently agrees on:

```text
total source points      = 238,146,391
total point payload      = 3,810,342,256 B
mean points / frame      = 793,821.3
mean point payload/frame = 12.701 MB
Camera point packets     = 2,976,979
mean packets / frame     = 9,923.3
```

The source-point total & binary payload therefore match the prepared 16-byte "Loot" representation documented in the dataset section.

The previously documented source PLY / BIN footprint comparison remains an **offline representation property**, not a streaming compression result. The current streaming validation begins from the prepared binary sequence & does not include converter time in any real-time latency figure.

### 18.2 Camera — Nominal 30-fps Source Operation & Local Backpressure

The current warm-mode `Camera` sustains the configured source cadence.

From consecutive `timestamp_start_tx` values:

```text
mean start-to-start interval = 33.330 ms
median interval              = 33.324 ms
95th percentile              = 34.143 ms
observed source rate         = 30.003 frames/s
```

Principal `Camera` timings are:

| Metric | Mean | Median | 95th percentile |
|---|---:|---:|---:|
| `disk_io_ms` | `1.966 ms` | `1.907 ms` | `2.467 ms` |
| `serialization_ms` | `1.714 ms` | `1.783 ms` | `1.929 ms` |
| `tx_duration_ms` | `11.930 ms` | `11.918 ms` | `12.967 ms` |
| `active_tx_ms` | `2.169 ms` | `2.175 ms` | `2.372 ms` |
| `active_process_ms` | `15.609 ms` | `15.543 ms` | `16.464 ms` |
| `total_residency_ms` | `15.610 ms` | `15.544 ms` | `16.465 ms` |
| `node_efficiency_pct` | `99.992 %` | `99.992 %` | `99.995 %` |

The logical & network-rate indicators average:

```text
internal_throughput_mbs ~= 1067.247 MB/s
logical_bitrate_mbps    ~= 3048.283 Mbit/s
network_bitrate_mbps    ~= 3243.564 Mbit/s
```

`CACHE_MODE_MIDDLE` still performs a timed `fread()` for every frame, but `WARM_MODE_ENABLED` has already made the source files resident before the measured sequence. The result must therefore be interpreted as a **warm file-backed source**, not as cold-storage performance.

The `Camera` observes substantial local Tx-ring pressure:

```text
mean tx_zero_accepts        = 12,379.96 / frame
mean tx_resubmitted_packets = 394,831.16 / frame
```

yet:

```text
tx_partial_accepts = 0
mbuf_starvation    = 0
status             = 1 for all 300 frames
```

and every expected point reaches the next stages. This is strong evidence that the bounded local resubmission strategy absorbs the measured producer / consumer mismatch without converting it into upstream application loss. It remains a local "DPDK" queue phenomenon & must not be described as "UDP" retransmission.

### 18.3 SFF1 / "GAC" — Cost of Moving Geometry into the Data Path

The validated run keeps `current_skip = 1`, so every source frame is admitted.

`SFF1` reports:

| Metric | Mean | Median | 95th percentile |
|---|---:|---:|---:|
| `geometry_aggregation_ms` | `2.084 ms` | `2.077 ms` | `2.147 ms` |
| `max_r_ms` | `1.067 ms` | `1.063 ms` | `1.101 ms` |
| `active_process_ms` | `4.066 ms` | `4.057 ms` | `4.289 ms` |
| `tx_duration_ms` | `19.908 ms` | `19.933 ms` | `21.140 ms` |
| `total_residency_ms` | `19.922 ms` | `19.947 ms` | `21.153 ms` |
| `node_efficiency_pct` | `20.420 %` | `20.415 %` | `21.128 %` |
| `camera_to_node_latency_ms` | `0.345 ms` | `0.366 ms` | `0.380 ms` |
| `network_jitter_ms` | `0.222 ms` | `0.080 ms` | `1.032 ms` |

The mean explicit geometric cost is therefore approximately:

```text
geometry_aggregation_ms + max_r_ms ~= 3.151 ms / frame
```

and it is now paid in the "GAC", where the information is generated while the point stream is already crossing the service path.

Integrity is exact in the measured run:

```text
Camera Tx points
= SFF1 Rx points
= SFF1 Tx points
= 238,146,391 points
```

with:

```text
data_integrity_pct = 100 % for all frames
eth_errors         = 0
ipv4_errors        = 0
udp_errors         = 0
nsh_errors         = 0
Tx resubmissions   = 0
```

The mean input throughput is approximately `674.766 MB/s`. The higher `SFF1` active cost relative to the preceding repository snapshot is intentional: exact radius work has been relocated from the `Encoder` into the "GAC" rather than eliminated from the system.

### 18.4 SFF2 Route 0 — Stateful Proxy Cost

For `SFF1 -> Encoder`, `SFF2` now performs service-header validation, proxy-state capture, decapsulation, plain-"UDP" reconstruction, route accounting, & forwarding.

Measured values are:

| Metric | Mean | Median | 95th percentile |
|---|---:|---:|---:|
| `active_process_ms` | `0.956 ms` | `0.908 ms` | `1.440 ms` |
| `tx_duration_ms` | `19.841 ms` | `19.875 ms` | `21.064 ms` |
| `total_residency_ms` | `19.847 ms` | `19.881 ms` | `21.071 ms` |
| `node_efficiency_pct` | `4.806 %` | `4.506 %` | `7.042 %` |
| `camera_to_node_latency_ms` | `0.434 ms` | `0.454 ms` | `0.477 ms` |
| `network_jitter_ms` | `0.226 ms` | `0.086 ms` | `1.021 ms` |

The route preserves all point data:

```text
SFF1 Tx points
= SFF2 Route-0 Rx points
= SFF2 Route-0 Tx points
= Encoder Rx points
```

for all 300 frames, with no reported protocol errors or Tx resubmissions.

The proxy therefore adds less than `1 ms` of mean measured active processing while removing the requirement for the `Encoder` to parse or maintain service-chain state.

### 18.5 Encoder — Geometry Offload, "GPU" Work, "Temporal" Controller, & "H.265"

The `Encoder` receives every expected point:

```text
data_integrity_pct = 100 % for all 300 frames
mbuf_starvation    = 0
Tx resubmissions   = 0
ffmpeg_write_eagain = 0
```

Because `OFFLOAD_MODE_ENABLED` is active & every frame receives a complete valid "GAC" snapshot:

```text
geometry_aggregation_ms = 0.000 ms for all 300 frames
max_r_ms                 = 0.000 ms for all 300 frames
```

This is the clearest telemetry evidence that geometry offload is operational: the corresponding exact geometric work is visible upstream in `SFF1` & absent from the `Encoder`'s complete-frame path.

Principal `Encoder` timings are:

| Metric | Mean | Median | 95th percentile |
|---|---:|---:|---:|
| `conversion_ms` | `3.350 ms` | `3.309 ms` | `3.462 ms` |
| `geometry_aggregation_ms` | `0.000 ms` | `0.000 ms` | `0.000 ms` |
| `max_r_ms` | `0.000 ms` | `0.000 ms` | `0.000 ms` |
| `projection_ms` | `4.122 ms` | `4.083 ms` | `4.204 ms` |
| `tx_duration_ms` | `8.635 ms` | `6.377 ms` | `15.504 ms` |
| `active_process_ms` | `16.108 ms` | `13.890 ms` | `29.815 ms` |
| `total_residency_ms` | `80.947 ms` | `81.388 ms` | `98.326 ms` |
| `node_efficiency_pct` | `20.269 %` | `17.066 %` | `32.812 %` |
| `encode_h265_ms` | `26.081 ms` | `24.270 ms` | `39.165 ms` |

"CUDA"-event decomposition is:

| Projection component | Mean |
|---|---:|
| `gpu_transfer_ms` | `1.468 ms` |
| `gpu_kernel_ms` | `0.768 ms` |
| `gpu_packing_ms` | `0.447 ms` |
| `gpu_copyback_ms` | `1.390 ms` |
| `host_overhead_ms` | `0.049 ms` |

The measured projection interval is therefore no longer dominated by host-to-device transfer to the extent observed in the preceding repository snapshot. Persistent buffers, larger "H2D" chunks, pinned output slots, static-pose analytic bounds, & fused "GPU" work all contribute to the current result.

The workload controller remains deliberately inactive under this load:

```text
"WARMUP" events         = 5
"IDLE" events           = 295
"SKIP+1" / "SKIP-1"     = 0
current_skip            = 1 for all frames
mean workload_ewma_ms   = 4.177 ms
mean workload_ratio     = 0.125
95th workload_ratio     = 0.127
frame_backlog           = 0 for all frames
mean codec_backlog      = 1.107 frames
mean wait_raw_queue_ms  = 0.003 ms
```

This is a useful negative result: the controller is integrated & observable, but the current run does not artificially trigger temporal degradation when the measured processing condition is healthy. A separate overload experiment is required to validate `SKIP+1` / recovery dynamics.

The current partial `Camera`-to-`Encoder`-egress latency is:

```text
end_to_end_latency_ms
mean   = 80.958 ms
median = 81.412 ms
p95    = 98.221 ms
```

Frame 300 reaches a larger tail value during final "codec" drain. Because current residency ends at encoded "DPDK" egress, these figures are intentionally broader than the `Encoder` residency values reported by the earlier repository snapshot & must not be compared directly with an "FFmpeg"-input-only boundary.

### 18.6 Encoder -> SFF2 Route 1 — Compressed-Media Integrity

The `Encoder` produces:

```text
"MPEG-TS" bytes = 10,345,640 B
media packets = 7,992
```

and Route 1 reports exactly:

```text
Rx media bytes = Tx media bytes = 10,345,640 B
Rx packets     = Tx packets     = 7,992
```

with `data_integrity_pct = 100 %` for every frame & zero protocol / Tx errors.

The compressed output-rate indicators agree between `Encoder` & Route 1:

```text
mean logical_bitrate_mbps ~= 8.298 Mbit/s
mean network_bitrate_mbps ~= 9.108 Mbit/s
```

The first compressed-media arrival at `SFF2` is observed at:

```text
camera_to_node_latency_ms
mean   = 51.835 ms
median = 48.903 ms
p95    = 65.138 ms
```

The Route-1 relay itself performs extremely little active "CPU" work:

```text
mean active_process_ms ~= 0.0038 ms
```

Its frame residency & throughput distributions are burst-dependent because encoded "MPEG-TS" bytes are emitted asynchronously rather than as a uniform point-packet train. In particular, the last frame is affected by "codec" drain. For this route, byte equality & first-media timing are more informative than treating the mean instantaneous receive throughput as a stationary link-capacity estimate.

### 18.7 "FFmpeg" / "NVENC" Stream Characteristics & Pre-Roll

The representative `ffmpeg.txt` contains:

```text
315 coded entries total
15 private pre-roll frames
300 application frames
```

Across all 315 entries:

```text
I frames = 21
P frames = 294
B frames = 0
```

Removing the single pre-roll I-frame & 14 pre-roll P-frames, the 300 real frames contain:

```text
I frames = 20
P frames = 280
B frames = 0
```

which is consistent with the configured 15-frame "GOP".

The real application coded-frame payload reported by "FFmpeg" is:

```text
10,051,241 B
```

while the `Encoder` emits:

```text
10,345,640 B "MPEG-TS"
```

The difference is:

```text
294,399 B ~= 2.93 % of the coded-frame bytes
```

and reflects transport-stream / muxing overhead rather than an application data-integrity mismatch.

The final "FFmpeg" cumulative statistic is approximately:

```text
avg_br ~= 7.688 Mbit/s
```

across the complete 315-frame timeline including the tiny blank pre-roll frames. The configured `10M` value remains a rate-control target, not a guarantee that the measured complete sequence will equal exactly `10 Mbit/s`.

The ratio between raw point payload delivered by the `Camera` & emitted "MPEG-TS" bytes is approximately:

```text
3,810,342,256 / 10,345,640 ~= 368.3 : 1
```

This is a **system data-volume ratio** across two different representations. It is not presented as a formal point-cloud "codec" compression ratio or a rate-distortion result.

### 18.8 Current Strengths & Current Limitations

The present upstream snapshot demonstrates simultaneously:

```text
30-fps Camera source cadence under the warm middle-cache condition
100 % point integrity across Camera -> "GAC" -> SFF2 -> Encoder
100 % compressed-media byte integrity across Encoder -> SFF2 Route 1
in-path geometric aggregation with exact final max_r
zero local Encoder geometry work when offload metadata are valid
stateful "NSH" proxying around an unaware Encoder
sub-millisecond mean active SFF2 Route-0 proxy work
low-millisecond "CUDA" projection
persistent pre-rolled "H.265" / "NVENC" operation
workload-driven temporal control integrated through SFF2 / SFF1 to Camera
zero measured frame backlog under the validated load
```

The snapshot does **not** yet establish:

```text
complete Decoder reconstruction
validated Decoder -> SFF3 packet semantics
Route-2 telemetry
operational User -> SFF3 -> SFF2 -> Decoder pose feedback
final Camera-to-User latency
visual / geometric reconstruction quality
controller behaviour under forced overload / recovery
multi-host or generic RFC-8300 interoperability
```

### 18.9 Improvement over the Preceding Validated Repository Snapshot

The current redesign produces a substantial improvement over the preceding repository snapshot on metrics whose semantic boundary remains sufficiently comparable.

| Comparable Quantity | Preceding Snapshot | Current Snapshot | Change |
|---|---:|---:|---:|
| Observed Camera rate | `~8.08 fps` | `~30.003 fps` | `3.71 x` ( `+271.3 %` ) |
| Camera `disk_io_ms` | `114.279 ms` | `1.966 ms` | `-98.3 %` |
| Camera `total_residency_ms` | `123.778 ms` | `15.610 ms` | `-87.4 %` |
| Encoder `conversion_ms` | `7.748 ms` | `3.350 ms` | `-56.8 %` |
| Encoder `projection_ms` | `15.661 ms` | `4.122 ms` | `-73.7 %` |
| Encoder "FFmpeg"-input `tx_duration_ms` | `11.330 ms` | `8.635 ms` | `-23.8 %` |
| Encoder `active_process_ms` | `34.738 ms` | `16.108 ms` | `-53.6 %` |
| Encoder `encode_h265_ms` | `259.894 ms` | `26.081 ms` | `-90.0 %` |
| Route-1 `camera_to_node_latency_ms` | `407.272 ms` | `51.835 ms` | `-87.3 %` |

Two increases must be interpreted as deliberate architectural work relocation rather than as unexplained regressions:

```text
SFF1 active_process_ms : 2.255 -> 4.066 ms
SFF2 R0 active_process : 0.841 -> 0.956 ms
```

`SFF1` now performs the geometry aggregation & exact `max_r` work that the architecture intentionally removes from the `Encoder`. `SFF2` Route 0 now performs stateful proxy capture / decapsulation rather than simple forwarding. The additional cost is therefore associated with new functionality.

The `Encoder` telemetry provides a direct offload marker:

```text
SFF1 geometry + max_r ~= 3.151 ms / frame
Encoder geometry      = 0.000 ms / frame
Encoder max_r         = 0.000 ms / frame
```

However, the full `Encoder` speed-up must **not** be attributed solely to this relocation. The current snapshot also introduces persistent "CUDA" resources, `H2D_CHUNK_POINTS = 65536`, static-pose analytic bounds, a fused projection path, three registered "I420" slots, a dedicated writer, "codec" pre-roll, & lower-latency "FFmpeg" / "NVENC" operation.

The current implementation therefore demonstrates a clear engineering advantage over the repository's preceding validated design & reaches a substantially more favourable operating point than the application-level architecture that motivated the redesign. Cross-work comparisons remain **indicative rather than controlled benchmarks** unless hardware, transport semantics, frame boundaries, & measurement definitions are matched explicitly.

A particularly important methodological point is that the current `Encoder` residency is **not** included in the comparison table. Its endpoint has moved from an earlier "FFmpeg"-input-oriented boundary to the last encoded "DPDK" egress, so a raw before / after percentage would be misleading.

### 18.10 Complete Statistical View of the Representative Telemetry

The following tables are generated directly from the representative 300-row CSV files. They are included deliberately so that the repository can be used as a measurement guide rather than exposing only a hand-picked subset of favourable indicators.

> **Interpretation rule:** cumulative protocol-error snapshots are reported using their final / maximum observed value rather than row sums. Tx resubmission & writer counters are genuinely frame-local & can therefore be summed. Absolute timer timestamps are documented in the telemetry dictionaries above but are not statistically averaged because their magnitude has no standalone performance meaning.

#### Camera

`status = 1`: **300 / 300**; `current_skip`: **1 -> 300 frames**.

| Metric | Mean | Median / Final | P95 / Max | Maximum / Total |
|---|---|---|---|---|
| `tx_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `tx_packets` | 9,923.263 | 9,874.000 | 10,219.050 | 10,444.000 |
| `payload_bytes` | 12,701,140.853 | 12,638,032.000 | 13,080,028.000 | 13,367,328.000 |
| `internal_throughput_mbs` | 1,067.247 | 1,064.210 | 1,144.624 | 1,197.250 |
| `logical_bitrate_mbps` | 3,048.283 | 3,033.137 | 3,139.217 | 3,208.168 |
| `network_bitrate_mbps` | 3,243.564 | 3,227.448 | 3,340.318 | 3,413.697 |
| `disk_io_ms` | 1.966 | 1.906 | 2.467 | 2.575 |
| `serialization_ms` | 1.714 | 1.783 | 1.929 | 1.983 |
| `tx_duration_ms` | 11.930 | 11.918 | 12.967 | 13.498 |
| `active_tx_ms` | 2.169 | 2.175 | 2.372 | 2.452 |
| `active_process_ms` | 15.609 | 15.543 | 16.464 | 16.973 |
| `total_residency_ms` | 15.610 | 15.544 | 16.465 | 16.974 |
| `node_efficiency_pct` | 99.992 | 99.992 | 99.995 | 99.997 |
| `tx_zero_accepts` | 12,379.960 | 12,218.000 | 14,255.000 | sum = 3,713,988 |
| `tx_partial_accepts` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_resubmit_calls` | 12,379.960 | 12,218.000 | 14,255.000 | sum = 3,713,988 |
| `tx_resubmitted_packets` | 394,831.157 | 389,248.500 | 454,915.000 | sum = 118,449,347 |
| `mbuf_starvation` | 0.000 | 0.000 | 0.000 | sum = 0 |

#### SFF1 / "GAC"

`status = 1`: **300 / 300**; `current_skip`: **1 -> 300 frames**.

| Metric | Mean | Median / Final | P95 / Max | Maximum / Total |
|---|---|---|---|---|
| `original_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `rx_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `tx_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `rx_packets` | 9,923.263 | 9,874.000 | 10,219.050 | 10,444.000 |
| `tx_packets` | 9,923.263 | 9,874.000 | 10,219.050 | 10,444.000 |
| `payload_bytes` | 12,701,140.853 | 12,638,032.000 | 13,080,028.000 | 13,367,328.000 |
| `data_integrity_pct` | 100.000 | 100.000 | 100.000 | 100.000 |
| `internal_throughput_mbs` | 674.766 | 672.360 | 707.873 | 743.697 |
| `logical_bitrate_mbps` | 3,048.283 | 3,033.137 | 3,139.217 | 3,208.168 |
| `network_bitrate_mbps` | 3,376.932 | 3,360.155 | 3,477.661 | 3,554.064 |
| `tx_duration_ms` | 19.908 | 19.933 | 21.140 | 21.552 |
| `active_tx_ms` | 0.656 | 0.648 | 0.784 | 0.810 |
| `active_process_ms` | 4.066 | 4.056 | 4.289 | 4.402 |
| `geometry_aggregation_ms` | 2.084 | 2.077 | 2.147 | 2.196 |
| `max_r_ms` | 1.067 | 1.063 | 1.101 | 1.278 |
| `cycle_ms` | 33.290 | 33.347 | 33.599 | 34.733 |
| `header_wait_ms` | 13.368 | 13.373 | 14.664 | 14.984 |
| `total_residency_ms` | 19.922 | 19.947 | 21.153 | 21.566 |
| `node_efficiency_pct` | 20.420 | 20.415 | 21.128 | 22.605 |
| `camera_to_node_latency_ms` | 0.345 | 0.366 | 0.380 | 0.475 |
| `schedule_delay_ms` | -0.346 | -0.236 | 0.027 | 0.090 |
| `network_jitter_ms` | 0.222 | 0.080 | 1.032 | 1.282 |
| `eth_errors` | cumulative | 0 | 0 | final / max |
| `ipv4_errors` | cumulative | 0 | 0 | final / max |
| `udp_errors` | cumulative | 0 | 0 | final / max |
| `nsh_errors` | cumulative | 0 | 0 | final / max |
| `tx_zero_accepts` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_partial_accepts` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_resubmit_calls` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_resubmitted_packets` | 0.000 | 0.000 | 0.000 | sum = 0 |

#### SFF2 Route 0

`status = 1`: **300 / 300**; `current_skip`: **1 -> 300 frames**.

| Metric | Mean | Median / Final | P95 / Max | Maximum / Total |
|---|---|---|---|---|
| `original_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `rx_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `tx_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `rx_media_bytes` | 0.000 | 0.000 | 0.000 | 0.000 |
| `tx_media_bytes` | 0.000 | 0.000 | 0.000 | 0.000 |
| `rx_packets` | 9,923.263 | 9,874.000 | 10,219.050 | 10,444.000 |
| `tx_packets` | 9,923.263 | 9,874.000 | 10,219.050 | 10,444.000 |
| `payload_bytes` | 12,701,140.853 | 12,638,032.000 | 13,080,028.000 | 13,367,328.000 |
| `data_integrity_pct` | 100.000 | 100.000 | 100.000 | 100.000 |
| `internal_throughput_mbs` | 640.645 | 638.266 | 670.649 | 702.078 |
| `logical_bitrate_mbps` | 3,048.283 | 3,033.137 | 3,139.217 | 3,208.168 |
| `network_bitrate_mbps` | 3,348.353 | 3,331.717 | 3,448.231 | 3,523.985 |
| `tx_duration_ms` | 19.841 | 19.875 | 21.064 | 21.471 |
| `active_tx_ms` | 0.732 | 0.696 | 1.095 | 1.340 |
| `active_process_ms` | 0.956 | 0.908 | 1.440 | 1.705 |
| `cycle_ms` | 33.290 | 33.347 | 33.599 | 34.744 |
| `header_wait_ms` | 13.442 | 13.462 | 14.733 | 15.044 |
| `total_residency_ms` | 19.847 | 19.881 | 21.071 | 21.478 |
| `node_efficiency_pct` | 4.806 | 4.505 | 7.042 | 8.014 |
| `camera_to_node_latency_ms` | 0.434 | 0.454 | 0.477 | 0.580 |
| `schedule_delay_ms` | -0.368 | -0.262 | 0.000 | 0.056 |
| `network_jitter_ms` | 0.226 | 0.085 | 1.021 | 1.297 |
| `eth_errors` | cumulative | 0 | 0 | final / max |
| `ipv4_errors` | cumulative | 0 | 0 | final / max |
| `udp_errors` | cumulative | 0 | 0 | final / max |
| `nsh_errors` | cumulative | 0 | 0 | final / max |
| `tx_zero_accepts` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_partial_accepts` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_resubmit_calls` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_resubmitted_packets` | 0.000 | 0.000 | 0.000 | sum = 0 |

#### Encoder

`status = 1`: **300 / 300**; `current_skip`: **1 -> 300 frames**; `event`: **IDLE -> 295, WARMUP -> 5**.

| Metric | Mean | Median / Final | P95 / Max | Maximum / Total |
|---|---|---|---|---|
| `original_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `rx_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `tx_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `rx_packets` | 9,923.263 | 9,874.000 | 10,219.050 | 10,444.000 |
| `tx_packets` | 26.640 | 22.000 | 103.050 | 111.000 |
| `payload_bytes` | 12,701,140.853 | 12,638,032.000 | 13,080,028.000 | 13,367,328.000 |
| `data_integrity_pct` | 100.000 | 100.000 | 100.000 | 100.000 |
| `internal_throughput_mbs` | 643.671 | 640.953 | 674.371 | 706.136 |
| `logical_bitrate_mbps` | 8.298 | 6.699 | 32.513 | 35.079 |
| `network_bitrate_mbps` | 9.108 | 7.364 | 35.706 | 38.521 |
| `conversion_ms` | 3.350 | 3.308 | 3.462 | 11.756 |
| `geometry_aggregation_ms` | 0.000 | 0.000 | 0.000 | 0.000 |
| `max_r_ms` | 0.000 | 0.000 | 0.000 | 0.000 |
| `projection_ms` | 4.122 | 4.083 | 4.204 | 11.761 |
| `tx_duration_ms` | 8.635 | 6.377 | 15.504 | 39.221 |
| `active_process_ms` | 16.108 | 13.889 | 29.815 | 46.829 |
| `total_processing_ms` | 16.108 | 13.889 | 29.815 | 46.829 |
| `total_residency_ms` | 80.947 | 81.388 | 98.326 | 340.138 |
| `node_efficiency_pct` | 20.269 | 17.066 | 32.812 | 51.899 |
| `gpu_transfer_ms` | 1.468 | 1.465 | 1.525 | 1.999 |
| `gpu_kernel_ms` | 0.768 | 0.765 | 0.790 | 0.911 |
| `gpu_packing_ms` | 0.447 | 0.444 | 0.463 | 0.520 |
| `gpu_copyback_ms` | 1.390 | 1.390 | 1.393 | 1.550 |
| `host_overhead_ms` | 0.049 | 0.024 | 0.025 | 7.436 |
| `camera_to_node_latency_ms` | 0.011 | 0.029 | 0.072 | 0.175 |
| `end_to_end_latency_ms` | 80.958 | 81.412 | 98.221 | 340.074 |
| `schedule_delay_ms` | 19.381 | 19.334 | 20.359 | 20.856 |
| `network_jitter_ms` | 0.228 | 0.085 | 1.061 | 1.316 |
| `wait_raw_queue_ms` | 0.003 | 0.002 | 0.012 | 0.018 |
| `wait_render_queue_ms` | 0.299 | 0.019 | 0.335 | 17.105 |
| `workload_ewma_ms` | 4.177 | 4.101 | 4.217 | 11.777 |
| `workload_ratio` | 0.125 | 0.123 | 0.127 | 0.353 |
| `frame_backlog` | 0.000 | 0.000 | 0.000 | 0.000 |
| `codec_backlog` | 1.107 | 1.000 | 2.000 | 3.000 |
| `encode_h265_ms` | 26.081 | 24.270 | 39.165 | 62.008 |
| `mpeg_bytes_generated` | 34,485.467 | 27,824.000 | 135,378.800 | 146,076.000 |
| `ffmpeg_write_calls` | 1.000 | 1.000 | 1.000 | sum = 300 |
| `ffmpeg_write_eagain` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_zero_accepts` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_partial_accepts` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_resubmit_calls` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_resubmitted_packets` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `mbuf_starvation` | 0.000 | 0.000 | 0.000 | sum = 0 |

#### SFF2 Route 1

`status = 1`: **300 / 300**; `current_skip`: **1 -> 300 frames**.

| Metric | Mean | Median / Final | P95 / Max | Maximum / Total |
|---|---|---|---|---|
| `original_points` | 793,821.303 | 789,877.000 | 817,501.750 | 835,458.000 |
| `rx_points` | 0.000 | 0.000 | 0.000 | 0.000 |
| `tx_points` | 0.000 | 0.000 | 0.000 | 0.000 |
| `rx_media_bytes` | 34,485.467 | 27,824.000 | 135,378.800 | 146,076.000 |
| `tx_media_bytes` | 34,485.467 | 27,824.000 | 135,378.800 | 146,076.000 |
| `rx_packets` | 26.640 | 22.000 | 103.050 | 111.000 |
| `tx_packets` | 26.640 | 22.000 | 103.050 | 111.000 |
| `payload_bytes` | 34,485.467 | 27,824.000 | 135,378.800 | 146,076.000 |
| `data_integrity_pct` | 100.000 | 100.000 | 100.000 | 100.000 |
| `internal_throughput_mbs` | 1,030.174 | 0.842 | 10,241.371 | 15,456.481 |
| `logical_bitrate_mbps` | 8.298 | 6.699 | 32.513 | 35.079 |
| `network_bitrate_mbps` | 9.108 | 7.364 | 35.706 | 38.521 |
| `tx_duration_ms` | 29.694 | 33.255 | 36.052 | 55.581 |
| `active_tx_ms` | 0.003 | 0.002 | 0.008 | 0.018 |
| `active_process_ms` | 0.004 | 0.003 | 0.011 | 0.023 |
| `cycle_ms` | 34.156 | 33.321 | 66.690 | 322.060 |
| `header_wait_ms` | 4.460 | 0.000 | 33.456 | 322.052 |
| `total_residency_ms` | 29.697 | 33.256 | 36.055 | 55.583 |
| `node_efficiency_pct` | 4.820 | 0.010 | 46.716 | 65.556 |
| `camera_to_node_latency_ms` | 51.835 | 48.903 | 65.138 | 340.635 |
| `schedule_delay_ms` | -7.950 | -10.877 | 4.651 | 280.257 |
| `network_jitter_ms` | 2.489 | 0.384 | 8.725 | 288.723 |
| `eth_errors` | cumulative | 0 | 0 | final / max |
| `ipv4_errors` | cumulative | 0 | 0 | final / max |
| `udp_errors` | cumulative | 0 | 0 | final / max |
| `nsh_errors` | cumulative | 0 | 0 | final / max |
| `tx_zero_accepts` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_partial_accepts` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_resubmit_calls` | 0.000 | 0.000 | 0.000 | sum = 0 |
| `tx_resubmitted_packets` | 0.000 | 0.000 | 0.000 | sum = 0 |

#### Cross-File Integrity & Source-Cadence Checks

```text
Camera mean start-to-start interval = 33.330057 ms
Observed Camera source rate         = 30.002949 frames/s
Total source points                 = 238,146,391
Total point payload                 = 3,810,342,256 B
Point integrity Camera -> SFF1 -> SFF2 Route 0 -> Encoder = 100 % for all 300 frames
Compressed media integrity Encoder -> SFF2 Route 1          = 100 % for all 300 frames
```

#### "FFmpeg" / "NVENC" `vstats` Cross-Check

The representative `ffmpeg.txt` contains **315 output statistics rows**: the private 15-frame pre-roll followed by 300 real application frames. The log therefore confirms that pre-roll is excluded from native application telemetry while still warming the persistent "codec" state.

```text
pre-roll rows                = 15
application rows             = 300
application I frames         = 20
application P frames         = 280
application encoded bytes    = 10,051,241 B
mean application frame size  = 33504.137 B
mean I-frame size            = 134773.050 B
mean P-frame size            = 26270.643 B
final reported average rate  = 7687.8 kbit/s
```


---


## ⚠️ 19. Experimental Interpretation & Known Boundaries

### 19.1 Logical Bytes vs. Wire Bytes

`logical_bitrate_mbps` & `network_bitrate_mbps` describe different accounting domains. Logical bitrate includes application data & one frame-level metadata instance as defined by the node; network bitrate accounts for the repeated output envelope on every packet.

They should not be merged into one generic "bitrate" value without stating the byte model.

### 19.2 `CACHE_MODE` & `WARM_MODE` Are Part of the Experimental Condition

The current source condition is:

```text
CACHE_MODE_MIDDLE
WARM_MODE_ENABLED
```

The `Camera` still executes per-frame `fread()`, but the source files are pre-mapped / locked for the run. The observed `~1.97 ms` `disk_io_ms` is consequently a warm file-backed measurement. A cold-cache or `CACHE_MODE_WORST` experiment is a different system condition & should not be compared without labelling it explicitly.

### 19.3 The Current Run Establishes the Upstream 30-fps Source Point, Not Final Real-Time "QoE"

The measured `Camera` cadence is approximately:

```text
30.003 frames/s
```

and the complete point path to the `Encoder` remains loss-free in the representative trace.

This is materially stronger than the preceding snapshot, but the final chain is still incomplete. A complete real-time claim requires `Decoder`, `SFF3`, `User`, rendering, & final latency / quality measurements.

### 19.4 Source Scheduling, Descriptor Depth, & Backpressure Are Joint Variables

The current `Camera` uses an absolute frame schedule rather than a separate pacing mode. Descriptor depth & bounded zero-accept resubmission determine how a selected frame's packet train interacts with local "vhost" / "OVS" queues.

The large `Camera` zero-accept counter is therefore an experimental observation about local queue pressure at the selected 30-fps point. Changing descriptor depth, "OVS" placement, retry bounds, or source cache condition defines a new experiment.

### 19.5 Core Affinity Is Part of the Experiment

Changing any of the following changes the execution environment:

```text
isolcpus state
"Docker" cpusets
"OVS" "lcore" placement
"OVS" "PMD" placement
"FFmpeg" "CPU" affinity
"CUDA" device / architecture target
```

The current results are inseparable from their "CPU" / "GPU" placement.

### 19.6 Route 2 Has Proxy Scaffolding but Undefined Application Semantics

`SFF2` already reserves Route 2 & contains the primary-path proxy step needed to re-impose base service metadata toward `SFF3` after a future `Decoder` return.

What is intentionally **not defined** is the `Decoder`'s final application packet format, frame-completion rule, payload-byte accounting, & corresponding Route-2 telemetry finalisation.

No correctness or performance claim is therefore made for:

```text
Decoder -> SFF2 Route 2 -> SFF3
```

until that contract is fixed.

### 19.7 Final E2E / "QoE" Is Not Yet Available

The `Encoder` metric named `end_to_end_latency_ms` currently reaches the last encoded "DPDK" egress attributed to the frame:

```text
Camera Tx -> Encoder compressed-media egress
```

This is broader than the earlier "FFmpeg"-input boundary but remains incomplete.

Final evaluation still requires:

```text
Decoder latency
geometric reconstruction
SFF3 behaviour
User rendering
"Pose" feedback delay
visual / geometric quality
command-to-photon timing
user-perceived "QoE"
```

### 19.8 "NSH" Interoperability Is Not Claimed

The repository now implements a more explicit `nsh_hdr` + context model & a stateful proxy around unaware functions, but it remains a closed experimental protocol. Generic "RFC 8300" interoperability is not claimed.

### 19.9 The "Temporal" Controller Was Not Stress-Activated in This Run

The controller is enabled, emits telemetry, & the reverse temporal chain is implemented, but all 300 source frames use `skip = 1` & no `SKIP+1` / `SKIP-1` event occurs.

The present trace validates **non-intrusive steady-state behaviour**, not overload adaptation. A separate controlled stress run is required to evaluate response delay, stability, oscillation resistance, & recovery hysteresis.

### 19.10 Binary Portability Has Two Separate Domains

The offline `.bin` file uses the converter's little-endian `float32` layout. The live "DPDK" path serialises implemented numeric network fields explicitly, including point-coordinate floating-point bit patterns.

A heterogeneous deployment must therefore standardise both the offline artefact format & any remaining application structures before assuming cross-machine binary compatibility.

### 19.11 Dataset Artefacts Are External to Git

The source PLY sequence & generated BIN sequence are not committed because each is multi-gigabyte.

Reproducibility therefore requires recording the dataset release, sequence, frame range, licence, scale factor, converter revision, & exact streaming source revision together with the telemetry.

---

## 🛠️ 20. Main Engineering Challenges & Current Solutions

### Limited Logical Cores

**Challenge:** the complete target chain contains more concurrent roles than can be assigned fully independent "CPU" resources on the present eight-logical-core host.

**Current approach:** one dedicated "OVS"-"DPDK" "PMD", explicit native-node affinity, "GPU" / "NVENC" offload, controlled sharing for inactive / auxiliary roles, & cooperative network servicing inside the `Encoder`.

### "DPDK" Backpressure at the Source

**Challenge:** one point-cloud frame contains approximately ten thousand application packets, so even a 30-fps source can temporarily exceed the acceptance rate of a local Tx ring.

**Current approach:** absolute source scheduling, large source / `Encoder` descriptor queues, bounded zero-accept resubmission, short pause backoff, & explicit per-frame backpressure counters. The current run retains full point integrity despite substantial `Camera` zero-accept activity.

### Sustaining the Nominal 30-fps Operating Point

**Challenge:** the preceding repository snapshot was source-I/O limited & operated near `8.08 fps`.

**Current approach:** `WARM_MODE_ENABLED` with the middle file-read strategy reduces timed source acquisition sufficiently for the `Camera` to maintain approximately `30.003 fps` in the current run. The next challenge is no longer merely reaching the source cadence; it is preserving the same operating point once `Decoder`, `SFF3`, `User`, dynamic pose, & forced temporal-controller stress are active simultaneously.

### Performing Useful Work In-Network

**Challenge:** not every geometric quantity is packet-local. Progressive sums / extrema are composable, but exact radius depends on the final centroid.

**Current approach:** `SFF1` performs the progressive pass during forwarding, stores only the frame-local `XYZ` needed for the unavoidable exact radius pass, & exports final geometry to the `Encoder`. This preserves the in-path objective without pretending that a mathematically frame-global quantity is available from the first packet.

### Avoiding Preprocessing That Would Invalidate the Data-Plane Question

**Challenge:** exact final geometry could be sent immediately only if another element computed it before the "GAC" observed the stream.

**Current approach:** reject that shortcut for the main experiment. `Camera` / offline preprocessing is limited to representation & transport preparation; the geometric service function remains the location where the geometric result is derived. The design therefore measures the cost of in-path computation rather than hiding it upstream.

### Protecting Rx While Computing

**Challenge:** a frame-aware `Encoder` can otherwise remain away from its "DPDK" Rx queue while performing local geometry, "H2D" transfer, "CUDA" work, or "codec" handoff.

**Current approach:** packet-arrival conversion, polling inside local fallback loops, `H2D_CHUNK_POINTS = 65536`, a "CUDA" polling callback while the stream is incomplete, three "I420" slots, & a dedicated writer thread. The current telemetry reports zero raw-frame backlog.

### Maintaining Service State Around Unaware Applications

**Challenge:** forcing every application to parse experimental "NSH" would couple service-chain research to `Encoder` / `Decoder` implementation details.

**Current approach:** `SFF2` captures primary state, removes service encapsulation before unaware functions, advances "SI" in proxy state, & re-imposes service metadata only when traffic returns to an aware boundary.

### Preserving "MTU" Across Different Traffic Types

**Challenge:** raw point packets, geometric service metadata, compressed media, temporal commands, & pose commands carry different envelopes.

**Current approach:** derive the payload constants from each complete packet format: `80 * 16 B` points upstream & `7 * 188 B` TS packets downstream, while keeping control packets compact.

### Correctly Attributing Asynchronous "Codec" Output

**Challenge:** "FFmpeg" pipe reads do not preserve video-frame boundaries & "codec" output is asynchronous relative to input submission.

**Current approach:** pre-roll one "GOP", reconstruct fixed TS packets, detect video "PES" starts, associate starts with the oldest submitted real frame, keep pre-roll private, & drain the "codec" before final telemetry is written.

---

## ✅ 21. Reproducibility Checklist

Every archived benchmark should preserve at least:

```text
source revision
README revision
host kernel
"CPU" model and logical "CPU" count
"GPU" model
NVIDIA driver
"CUDA" version
"DPDK" version
"Open vSwitch" version
"Docker" version
"FFmpeg" version

HugePage count and size
"OVS" "lcore" placement
"OVS" "PMD" placement
"OVS" socket memory
"Docker" cpusets
isolcpus state
"FFmpeg" "CPU" affinity

Camera CACHE_MODE
Camera WARM_MODE
TARGET_FPS
K_FRAMES
POINTS_PER_PACKET
BURST_SIZE
MAX_ZERO_ACCEPTS
Camera Rx / Tx descriptor depth

SFF1 "GAC" geometry layout
SFF1 workspace / MAX_FRAME_POINTS
SFF1 Rx / Tx descriptor depth
"Main" "SPI" / "SI" values
"Temporal" "SPI" / "SI" values
"Pose" "SPI" / "SI" values

SFF2 proxy rules
SFF2 Route-0 / Route-1 byte semantics
SFF2 Route-2 implementation state
SFF2 Rx / Tx descriptor depth

Encoder OFFLOAD_MODE
Encoder TEMPORAL_ADAPTATION
MAX_SKIP
EWMA_ALPHA
OVERLOAD_RATIO
RECOVERY_RATIO
OVERLOAD_STREAK
RECOVERY_STREAK
RETRY_FRAMES
H2D_CHUNK_POINTS
YUV_BUFFER_COUNT
MAX_POINTS
face / atlas dimensions
CAMERA_DISTANCE
"CUDA" warm-up state

"FFmpeg" / "NVENC" pre-roll length
target "H.265" bitrate
buffer size
preset / tune
"GOP" length
B-frame setting
lookahead / delay / zero-latency options
"MPEG-TS" payload size

"OVS" debug / mirror state
all native telemetry CSV files
"FFmpeg" statistics / stderr logs
```

For the current representative run, the defining source / `Encoder` conditions are:

```text
CACHE_MODE           = CACHE_MODE_MIDDLE
WARM_MODE            = WARM_MODE_ENABLED
OFFLOAD_MODE         = OFFLOAD_MODE_ENABLED
TEMPORAL_ADAPTATION  = TEMPORAL_ADAPTATION_ENABLED
TARGET_FPS           = 30
K_FRAMES             = 300
H2D_CHUNK_POINTS     = 65536
YUV_BUFFER_COUNT     = 3
```

A benchmark is reproducible only when the result files & the configuration that produced them are archived together.

---

## 🚧 22. Ongoing Work

The next work should remain divided into **functional completion**, **performance validation**, & **quality / end-to-end validation**.

### Functional Completion

```text
1. define the Decoder input / output application contract
2. implement "H.265" decoding and geometric reconstruction
3. finalise SFF2 Route-2 payload accounting and telemetry
4. implement SFF3 processing
5. implement User rendering / interaction
6. complete User -> SFF3 -> SFF2 -> Decoder "Pose" validation
7. retain Encoder / Decoder "NSH"-unaware operation through the SFF2 proxy
8. define final "EOS" semantics for the complete downstream path
```

### Performance Validation

```text
1. perform OFFLOAD_MODE_ENABLED vs OFFLOAD_MODE_DISABLED A/B runs
2. force controlled Encoder overload to validate SKIP+1 / RETRY / SKIP-1 dynamics
3. quantify temporal-controller response time, hysteresis, and oscillation behaviour
4. compare CACHE_MODE / WARM_MODE combinations as separate source conditions
5. study descriptor depth and Camera zero-accept backpressure sensitivity
6. repeat validated runs and report confidence intervals / dispersion
7. re-evaluate Core 0 sharing once Decoder "codec" work is active
8. measure complete-chain sustained 30-fps behaviour
9. evaluate static-pose "CUDA" assumptions once dynamic pose is introduced
10. isolate the contribution of geometry offload from the other Encoder optimisations
11. evaluate "FFmpeg" / "NVENC" rate-control and latency sensitivity
```

### Quality & End-to-End Validation

```text
1. Decoder reconstruction correctness
2. final Camera-to-User end-to-end latency
3. point-to-point geometric error / D1-style metrics where appropriate
4. point-to-plane / surface-aware geometric error where appropriate
5. texture / attribute quality
6. reconstructed occupancy consistency
7. bitrate-quality trade-off analysis
8. User pose / command-to-photon latency
9. final user-facing "QoE" evaluation
```

The immediate architectural dependency is the `Decoder` packet contract. Until that representation is fixed, Route-2 telemetry should remain intentionally incomplete rather than being populated with speculative byte semantics.

---

## 📚 23. References

1. J. Halpern & C. Pignataro, **"Service Function Chaining" ( "SFC" ) Architecture**, "RFC 7665", IETF, 2015.
2. P. Quinn, U. Elzur, & C. Pignataro, **"Network Service Header" ( "NSH" )**, "RFC 8300", IETF, 2018. The present project uses its "SPI" / "SI" terminology & architectural concepts but does not claim full "MD-Type-2" wire-format compliance.
3. E. d'Eon, B. Harrison, T. Myers, & P. A. Chou, **8i Voxelized Full Bodies — A Voxelized Point Cloud Dataset**, ISO/IEC JTC1/SC29 Joint WG11/WG1 input document WG11M40059/WG1M74006, Geneva, January 2017.
4. **JPEG Pleno Database**, *8i Voxelized Full Bodies ( 8iVFB v2 ) — A Dynamic Voxelized Point Cloud Dataset*. Dataset page: `https://plenodb.jpeg.org/pc/8ilabs/`.
5. **"DPDK" Project**, "Data Plane Development Kit" documentation, including Ethdev Rx / Tx queue APIs & "virtio-user" configuration.
6. **"Open vSwitch" Project**, "Open vSwitch" & "OVS"-"DPDK" documentation, including "DPDK" "vhost-user"-client ports.
7. **NVIDIA**, "CUDA" Toolkit documentation & NVIDIA Video "Codec" / "NVENC" documentation.
8. **"FFmpeg" Project**, "FFmpeg" documentation.
9. Maria Giovanna Lacaria, **Point Cloud Coding for Extended Reality Services**, Master's Thesis, Sapienza University of Rome, Academic Year 2025/2026. This reference is retained as the application-level comparison baseline documented by the project.

---

## Final Note

The principal contribution of this repository is the **co-design of packet transport, service steering, & computation**.

The point cloud is not merely carried between isolated applications. The `Camera` admits frames according to an `Encoder`-derived workload signal, the "GAC" performs geometry while packets are already traversing the data path, `SFF2` preserves experimental "NSH" state around unaware applications, & the `Encoder` combines packet-progressive reception with frame-level "CUDA" / "codec" processing.

The current 300-frame "Loot" snapshot materially advances the preceding repository design: the warm middle-cache source sustains approximately `30.003 fps`, point & compressed-media integrity remain complete across the validated path, `Encoder`-local geometry work is eliminated when "GAC" offload is valid, projection & first-"PES" latency are substantially reduced, & the workload-driven "Temporal" chain is integrated through to `Camera`-side admission.

The resulting platform is therefore best interpreted as an experimental study of **which volumetric-streaming operations can be executed in place on a software data plane, which frame-global dependencies must remain explicit, & how an application-aware processing bottleneck can regulate the source without relying on transport-level retransmission or user-driven quality control**.

The repository intentionally stops short of claiming full `Camera`-to-`User` real-time "QoE", a validated `Decoder`-to-`SFF3` application format, generic "RFC 8300" interoperability, or a controlled superiority result over heterogeneous reference platforms. Those claims require the downstream implementation & the additional validation stages listed above.