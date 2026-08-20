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

This repository serves as an experimental research platform rather than a production-ready "Service Function Chaining" ( "SFC" ) framework. The current snapshot implements & evaluates the upstream segment of the intended volumetric chain, while the downstream reconstruction & user-facing stages remain under active development.

| Node | Condition | Responsibility |
|---|---|---|
| `Camera` | Validated | "DPDK"-native point-cloud source, warm-mode file acquisition, frame packetisation, absolute scheduling, temporal selection, & initial telemetry |
| `SFF1` | Validated | "Geometry-Aware Classifier" ( "GAC" ) implementing packet-level spatial aggregation, exact frame-completing radius evaluation, experimental "Network Service Header" ( "NSH" ) metadata insertion, & control ( "Temporal" ) decapsulation directed to `Camera` |
| `SFF2`<br>( Route 0 ) | Validated | Stateful dispatcher & proxy for the `SFF1` -> `SFF2` -> `Encoder` path; removes outer encapsulation prior to "SFC"-unaware application elements, thereby preserving relevant application content |
| `Encoder` | Validated | "SFC"-unaware frame assembly, geometry-offload consumption or local fallback retrieval, workload-driven monitoring, "CUDA" projection, "FFmpeg" / "NVENC" pre-roll & encoding, "MPEG-TS" chunking & attribution, alongside "UDP" output segmentation |
| `SFF2`<br>( Route 1 ) | Validated | Proxy-maintained `Encoder` -> `SFF2` -> `Decoder` path ( "Main" ) transition & compressed-media relay directed towards the prospective `Decoder` |
| `Decoder` | Under<br>development | "H.265" decoding & spatial reconstruction; intended to remain "SFC"-unaware & receive an ordinary application datagram from the proxy boundary |
| `SFF2`<br>( Route 2 ) | Reserved, not operational | Upcoming `Decoder` -> `SFF2` -> `SFF3` passage. The application format & route-specific telemetry are intentionally deferred until the `Decoder` output contract is stabilised |
| `SFF3` | Under<br>development | Final data-path stage & user-side command entry point ( "Pose" ) |
| `User` | Under<br>development | Rendering, interaction, zoom generation, & client-perceived "Quality of Experience" ( "QoE" ) measurements |

The **presently sustained upstream chain** is represented horizontally as:

```
Camera -> SFF1 -> SFF2 ( Route 0 ) -> Encoder -> SFF2 ( Route 1 )
```

The **complete target primary route**, designated as **"Main"**, is structured as:

```
Camera -> SFF1 -> SFF2 ( Route 0 ) -> Encoder -> SFF2 ( Route 1 ) -> Decoder -> SFF2 ( Route 2 ) -> SFF3 -> User
```

Control mechanisms are deliberately separated into two independent logical service paths rather than consolidated into a single monolithic feedback packet:

```
"Temporal" : Encoder -> SFF2 -> SFF1 -> Camera
"Pose"     : User -> SFF3 -> SFF2 -> Decoder
```

The temporal loop is fully operational. `Encoder` determines a `temporal_skip`, `SFF2` classifies the plain datagram into the corresponding chain, `SFF1` eliminates the envelope, & `Camera` applies the requested factor prior to subsequent source-frame transmission. The representative validation run is maintained at `current_skip = 1`, as the `Encoder` workload does not satisfy the configured overload indicators.

"Pose" evolution is not yet active. Consequently, `yaw = 0`, `pitch = 0`, & `zoom = 1` serve as static references for the complete validated upstream course. However, the proposed design intends to originate updated information at the `User`, propagate it through `SFF3` & `SFF2`, & expose it to `Decoder` without requiring any intermediate node to assume responsibility for client-driven quality adjustments.

> **Repository Note:** This README documents the native telemetry exported by `Camera`, `SFF1`, `SFF2`, & `Encoder`, alongside the offline Converter & independent "FFmpeg" statistics utilised for cross-verification.

> **Validation Scope:** The quantitative results delineated below refer to the representative 300-frame "Loot" experiment using the precise compile-time & launcher configurations present in this snapshot. Specifically, `Camera` employs `CACHE_MODE_MIDDLE` in conjunction with `WARM_MODE_ENABLED`. `Encoder` utilises `OFFLOAD_MODE_ENABLED` & `TEMPORAL_ADAPTATION_ENABLED`. The measured `Camera` start-to-start interval is approximately `33.330 ms`, corresponding to roughly `30.003 frames/s`. While this result establishes the nominal upstream source cadence for the measured configuration, it does not constitute a complete `Camera`-to-`User` real-time proof, as `Decoder`, `SFF3`, & `User` remain unimplemented.

---

## 🎯 1. Project Motivation & Research Objective

This project investigates the feasibility of migrating selected functions within a real-time volumetric streaming pipeline from conventional application-level microservices to direct execution during packet traversal through a "DPDK"-based "SFC".

The objective extends beyond merely substituting kernel sockets with a faster packet-I / O "API". The principal research question addresses whether the forwarding path can function as an active computational component without overloading the data plane or compromising the semantic correctness of frame-level processing. This approach aims to preserve bounded queuing behaviour, data integrity, & sufficient observability to accurately attribute latency to the responsible components.

### 1.1 Why "DPDK" & "UDP" Are Topological Requirements

"DPDK" is employed because the experiment necessitates more than aggregate throughput. Service functions require the capability to inspect & modify the packet envelope directly, retain per-frame state, perform incremental computations while traffic is in flight, observe local queue acceptance, & account separately for active `rte_eth_tx_burst()` execution versus wall-clock backpressure. A conventional kernel-socket path would deliberately obscure portions of the packet lifecycle behind socket queues, scheduler wake-ups, generic buffering, & transport-stack policies. While valuable for general-purpose applications, such mechanisms would blur the principal experimental boundaries.

"UDP" is equally requisite. Volumetric frames inherently carry their own identity, packet sequence, original-point count, & temporal relevance. Consequently, the application benefits from preserving datagram boundaries & explicitly determining which information retains utility. A reliable, ordered byte stream such as "TCP" would couple subsequent data to the retransmission & in-order delivery of prior missing bytes, thereby introducing head-of-line blocking precisely when a delayed volumetric frame holds less value than a more recent one. Furthermore, it would shift flow control & recovery policies into the transport layer, whereas this project intentionally investigates source-rate regulation via an application-aware temporal controller.

This architectural choice does **not** assume "UDP" reliability. Instead, correctness is rendered observable through explicit frame & packet identifiers, point / media counters, integrity percentages, protocol field validation, redundant "End-of-Stream" ( "EOS" ) signalling, & bounded local resubmissions when a Tx ring temporarily accepts zero elements.

**Accordingly**, the architecture leverages "DPDK" & "UDP" to achieve:

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

Performance is a natural consequence of the design; however, **experimental control, packet semantic visibility, & the avoidance of transport-level head-of-line coupling serve as equally pivotal motivations**.

### 1.2 Selecting Computation for the Data Plane

The design delineates operations according to their underlying mathematical dependency structures.

Operations that are **associative, incrementally composable, & element-progressive** present as natural candidates for in-network execution ( "GAC" ):

```
sum_x, sum_y, sum_z
min_x, min_y, min_z
max_x, max_y, max_z
active_point_count
```

From these running quantities, `SFF1` extracts significant metrics, including:

```
C_N = ( 1 / N ) * sum_i( p_i )
E_N = p_max,N - p_min,N
B_N = ( p_min,N + p_max,N ) / 2
```

where `C_N` denotes the centroid of the observed points, while `E_N` & `B_N` represent the current axis-aligned extent & bounding-box centre, respectively.

Certain geometric entities, such as the maximum outer radius, cannot achieve exactitude prior to frame completion:

```
max_r = max_i || p_i - C_final ||_2
```

This metric is contingent upon `C_final`, which remains undetermined until the frame is entirely received. Consequently, "GAC" maintains a compact frame-local `XYZ` workspace, executing the initial aggregation pass while packets are actively forwarded, & performs the precise distance calculation only upon ascertaining the final population. The resultant data is then appended to the scene-completing packet.

This limitation is not merely a deferred optimisation resolvable by transmitting an alternative packet earlier. Pre-computing the accurate centroid-dependent boundary at the `Camera`, within an offline stage, or via another processing element would shift the computation outside the data-plane locale under experimental evaluation. Similarly, pose-dependent transformed frontiers cannot be finalised until the requisite stance data is available. Thus, the implementation rigorously distinguishes between **continuous in-path information** & **frame-global information mathematically unavailable until a specific barrier is breached**.

Procedures necessitating the complete active point set, final reconstruction parameters, "GPU" visibility, or "codec" state are retained downstream:

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

The `Encoder` operates neither as a conventional isolated application nor as a purely stateless data-plane function. It retains a frame-completion boundary prior to projection, yet it incrementally performs packet-by-packet conversion & placement, consumes progressive / final geometric metadata, cooperatively services "DPDK" during "CPU" / "GPU" workloads, & decouples "codec" input via a writer queue. It functions as a **hybrid frame-aware elaboration node** situated between application-level semantics & data-plane-oriented incremental execution.

### 1.3 Connection with the Reference Pipeline

This methodical approach is informed by state-of-the-art literature, including baseline models & works previously cited in this repository, which offer a comprehensive volumetric streaming mechanism & a robust performance-evaluation framework.

The current project does **not** replicate that architecture verbatim. Instead, it reformulates the corresponding workload around:

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

Consequently, comparisons with the reference implementation are valid only at **semantically equivalent frontiers**. Exact numerical parity is neither anticipated nor methodologically sound given variations in transport mechanisms, buffering, node boundaries, "CPU" placement, cache residency, "GPU" kernels, or "codec" queuing policies.

### 1.4 Guidelines & "Codec" Scope

Two distinctions remain critical for accurate interpretation of this work.

Firstly, the project integrates the **"SFC" concepts** of "Service Path Identifier" ( "SPI" ) & "Service Index" ( "SI" ), alongside an experimental "MD-Type-2"-like context layout. Nevertheless, the wire representation constitutes a closed project protocol & is appropriately characterised as **"NSH"-inspired**, rather than asserting generic "RFC 8300" interoperability.

Secondly, `Encoder` does **not** implement "MPEG" "V-PCC" or "G-PCC". It constructs a custom 6-view "Geometry" / "Texture" / "Occupancy" "Atlas", employing "HEVC" ( `hevc_nvenc` ) as the video compression engine for this representation. The resulting bitrate & latency metrics characterise the specific projection-&-video path & should not be presented as standards-compliant coding benchmarks.

---

## 🧩 2. Architectural Roles & Why Each Node Matters

| Device | Importance |
|---|---|
| `Camera` | Establishes the absolute timeline, reads prepared fixed-width frames, serialises coordinates into network byte order, packetises below the "MTU", & applies the most recent `temporal_skip` before frame injection. This prevents superfluous elements from consuming downstream resources when temporal relief is requested. |
| `SFF1` | Demonstrates the core in-path computation principle. It calculates progressive centroid / extent / bounding-box metrics during forwarding & the exact final `max_r` upon frame completion, exporting results via experimental service metadata. Additionally, it decapsulates the "Temporal" control chain without independently deciding frame omissions. |
| `SFF2` | Segregates traffic steering from service-function awareness. Functions as a 4-port forwarder & stateful "NSH" proxy, enforcing the primary & both reverse control paths. Capable boundaries utilise "SPI" / "SI" semantics, whereas Encoder & the future Decoder operate on standard "UDP" application packets. |
| `Encoder` | Reconstructs complete frames while incrementally processing packet-arrival conversions. Validates & consumes cumulative metadata produced by SFF1 ( when offload is active ), executes local calculations as fallbacks, drives the "CUDA" projection pipeline, feeds a persistent pre-rolled "NVENC" process, attributes asynchronous "MPEG-TS" outputs to source elements, & regulates temporal loads based on processing conditions. |

"OVS"-"DPDK" remains deliberately immediate, providing deterministic adjacency between "vhost-user" interfaces alongside a "Default-Deny" policy substrate. It performs no point-cloud computations. Service functions are the sole components responsible for application-specific data-plane operations.

The architectural boundary is thus clearly demarcated:

```
"OVS"-"DPDK"    -> deterministic virtual adjacency
"SFF1" / "SFF2" -> route computation & steering
"Encoder"       -> hybrid frame / "GPU" / "codec" processing
"Camera"        -> workload admission through temporal selection
```

---

## 🔗 3. Service-Chain Semantics

### 3.1 "Main" Service Path

The project defines:

```
MAIN_SPI = 100
```

The "SI" model is preserved by `SFF2` even when adjacent application functions lack "SFC" awareness. Primary transitions include:

```
SFF1 emits aware state                         : "SPI 100", "SI 255" + geometry context
SFF2 captures state & removes service envelope : Encoder receives plain "UDP" + geo_agg_hdr
Encoder returns plain compressed traffic       : proxy state advances "SI 255 -> 254"
SFF2 forwards plain application traffic        : Decoder remains "SFC"-unaware
Future Decoder return                          : proxy state advances "SI 254 -> 253"
SFF2 re-imposes base "NSH" toward SFF3         : "SPI 100", "SI 253"
```

The fully validated segment is:

```
Camera -> SFF1 ( "GAC", aware boundary )-> SFF2 ( proxy capture / decapsulation ) -> Encoder ( unaware ) -> SFF2 ( proxy state transition ) -> Decoder ( unaware )
```

`SFF2` ( Route 2 ) incorporates the fundamental scaffolding required to inject the service header toward `SFF3`, though the `Decoder`-side packet contract remains uncommitted. Semantics & telemetry are intentionally deferred to prevent reliance on an unstable format.

### 3.2 "Temporal" Service Path

"Temporal" adaptation utilises:

```
TEMPORAL_SPI = 200
TEMPORAL_SI  = 255
```

`Encoder` generates this decision; within the current scenario, the primary modification target is **elaboration capacity**, not user-selected visual quality.

```
Encoder
  -> standard "UDP" payload
  -> SFF2 classifies + imposes service metadata
  -> SFF1 validates + removes the outer encapsulation
  -> ...
  -> Camera updates temporal skip
```

The structural layout is:

```
frame_id : uint32
skip     : uint16
padding  : uint16
```

Upon adopting a value, the `Camera` filters source frames based on the active factor prior to segmentation. The resulting nominal active-element rate is:

```
FPS = TARGET_FPS / skip
```

This structural arrangement is pivotal. If an overloaded `Encoder` discarded frames post-transit through `Camera`, `SFF1`, & `SFF2`, upstream bandwidth & computational resources would be wasted. Reverting the decision to the `Camera` transforms the controller into an admission mechanism for the stream's subsequent segments.

### 3.3 "Pose" Service Path

"Pose" control is distinct from temporal regulation:

```
POSE_SPI = 300
POSE_SI  = 255
```

The intended flow is:

```
User -> SFF3 -> SFF2 -> Decoder
```

At the `SFF2` boundary, an "NSH"-encapsulated pose payload is stripped & redirected as plain "UDP" to `Decoder`. The current upstream run bypasses this chain, as `Decoder`, `SFF3`, & `User` interactions remain in development.

Consequently, `yaw`, `pitch`, & `zoom` remain static across the validated `Camera` to `Encoder` pathway. Dynamic user stances are slated to become a **downstream reconstruction or rendering concern**.

### 3.4 Protocol Clarification

The architecture employs an 8-byte `nsh_hdr`, the "SPI" / "SI" paradigm, a "Time-to-Live" ( "TTL" ) field, & a defined context for geometric metadata. `SFF2` sustains proxy state while service functions remain agnostic to the chain envelope.

Nevertheless, the implementation is accurately described as **experimental / "NSH"-inspired**, not as a universally interoperable "RFC 8300" stack. The spatial container operates as a fixed contract among participating modules, & the chosen `next_protocol` / information conventions are interpreted strictly within this repository's context.

Service-chain endpoints predominantly utilise the project-designated "UDP" port `6633`, while the source-facing Camera adjacency maintains its dedicated ports. The current address contract is:

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

These IP allocations define an explicit closed-testbed contract rather than a dynamically routed deployment model. "OVS"-"DPDK" provides the physical virtual adjacency, while native functions construct & validate the corresponding "Ethernet" / "IPv4" / "UDP" envelopes.

When an "IPv4" header is generated or rewritten, its checksum is cleared & recomputed via `rte_ipv4_cksum()`. The current "IPv4" framework deliberately sets the "UDP" checksum to zero. While valid within this testbed, this practice must not persist into future "IPv6" iterations, where zeroed "UDP" checksums contravene standard endpoint rules.

The fundamental capability demonstrated here is the evaluation of conditions, in-path elaboration, proxy decapsulation / re-encapsulation, & "SFC"-unaware component integration within a unified, controlled protocol definition.

---

## 📦 4. Data Representation & Packet Formats

### 4.1 Endianness & Portability

Offline ( `.bin` ) & network ( live ) representations serve distinctly different roles by design.

The `Converter.py` script constructs a contiguous 16-byte point data structure employing "Little-Endian" `float32` coordinates:

```
"x", "y", "z" -> "<f4"
"r", "g", "b" -> "u1"
"padding"     -> "u1"
```

Prior to network transmission, `Camera` reinterprets each "IEEE-754" value as a 32-bit word, converting the bit pattern to network byte order. `SFF1` reverses this operation for geometric computation & re-encodes coordinates when exporting metadata. `Encoder` executes the network-to-host shift upon receiving the point stream & spatial context.

Consequently:

- protocol fields are serialised on the "DPDK" path;
- geometric details adhere to the explicit bit-pattern convention;
- the file remains a host-preparation artefact strictly following the documented layout.

This architecture eliminates ambiguity between storage & network formats. The on-wire representation is explicit for implemented components, whereas the offline result adheres to the Converter specification governing this experiment.

### 4.2 Common Structures

| Structure | Size | Function |
|---|---:|---|
| `point_tx` | `16 B` | `x`, `y`, `z` as `float32`, "RGB" as `uint8`, plus an additional `padding` byte |
| `cam_hdr` | `40 B` | Frame identity, packet sequence, Camera timestamp, static pose reference, temporal skip, original-point count, & points in the packet |
| `nsh_hdr` | `8 B` | Service-chain base conveying "SPI", "SI", "TTL", metadata type, & next-protocol fields |
| `nsh_md2_ctx_hdr` | `4 B` | Project geometric-context descriptor |
| `geo_agg_hdr` | `44 B` | Centroid, extent, bounding-box centre, precise / progressive `max_r`, & active-point count |
| `enc_hdr` | `48 B` | Media packet identifier, scale, pose-compatibility, & reconstruction information |
| `temporal_payload` | `8 B` | Source frame associated with the control decision & requested adjustment |
| `pose_payload` | `12 B` | `yaw`, `pitch`, & `zoom` values for the User to Decoder path |

The project has superseded the previous fixed `int_hdr` sums / extrema block for downstream geometry contracts. `SFF1` now transmits directly usable quantities via `geo_agg_hdr`.

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

The explicit padding byte guarantees a deterministic 16-byte record shared seamlessly between the Converter & native devices. This layout facilitates fixed-offset packet parsing & aligned host-side storage; however, it does not inherently imply global vectorisation ( e.g., via "SIMD" executions ).

### 4.4 Camera Header

The 40-byte `cam_hdr` persists across the upstream point path, conveying vital frame-local details:

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

`frame_id` identifies the original source frame, retaining meaning even when intermediate elements are omitted via temporal selection. `sequence_number` designates the `Camera` packet position. `original_points` defines the total expected size, while `points_in_packet` records the contribution of the current datagram, enabling receivers to reconstruct frames without relying on external indices.

The initial pose is deliberately static:

```
yaw   = 0.0
pitch = 0.0
zoom  = 1.0
```

Dynamic stance adjustments are reserved for the independent "Pose" service path.

### 4.5 Geometric Metadata Produced by SFF1

The "GAC" exports:

```
centroid_x, centroid_y, centroid_z
extent_x, extent_y, extent_z
bbox_center_x, bbox_center_y, bbox_center_z
max_r
active_point_count
```

For an observed point prefix `P_N`, the progressive quantities are derived as:

```
C_N = ( 1 / N ) * sum_{ p in P_N }( p )
E_N = p_max,N - p_min,N
B_N = ( p_min,N + p_max,N ) / 2
```

Intermediate elements convey the most accurate snapshot currently available. Upon the arrival of the frame-completing packet, `C_N`, `E_N`, & `B_N` crystallise, & `max_r` is computed precisely as:

```
max_r = max_{ p in P_frame } || p - C_final ||_2
```

`active_point_count` enables `Encoder` to verify if the received metadata comprehensively describes the active point set. With `OFFLOAD_MODE_ENABLED`, valid metadata precludes the need for local geometry aggregation & radius computation.

### 4.6 Encoder Structure

The 48-byte `enc_hdr` accompanies compressed media exiting the node:

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

`packet_id` resets for each encoded frame attributed from the "MPEG-TS" / "PES" stream. The remaining fields conserve the geometric parameters requisite for `Decoder` to execute the projected representation.

`yaw` & `pitch` are retained for packet-format compatibility but remain static in the validated iteration. Dynamic pose metadata is anticipated to arrive independently via the "Pose" service chain.

---

## 📐 5. "MTU"-Aware Packet Design

The system architectures normal packets around a standard limit:

```
"IPv4" "MTU" = 1500 B
```

All implemented application packets are carefully sized to keep the complete datagram beneath this threshold. "Eth" framing is calculated separately, as the "MTU" excludes its 14-byte header.

### 5.1 Camera -> SFF1

`Camera` transmits a maximum of:

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
"Eth" frame              1362 B ( excluding "FCS" / preamble )
"IP"-"MTU" margin         152 B
```

The threshold of `80` is intentionally conservative, ensuring sufficient headroom for the supplemental service metadata inserted by `SFF1` rather than saturating the "MTU" to its theoretical maximum.

### 5.2 SFF1 -> SFF2 -> Encoder

`SFF1` strips the preceding "Eth" / "IPv4" / "UDP" network envelope, conducts spatial aggregation, & prepends its distinct service block:

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

At Route 0, `SFF2` functions as an "NSH" proxy. It decapsulates the `nsh_hdr` + context layer prior to reaching the unaware `Encoder`, while **preserving `geo_agg_hdr` as application-visible geometry**:

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

This ensures metadata remains accessible for service-path processing without intruding into "SFC"-unaware parsers.

### 5.3 Encoder -> SFF2 -> Decoder

The `Encoder` consolidates complete "MPEG-TS" chunks, preventing fragmentation across multiple datagrams. Packets are fixed at:

```
TS_PACKET_SIZE   = 188 B
MTU_PAYLOAD_SIZE = 7 * 188 = 1316 B
```

The maximum encoded-media size is:

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

`Encoder` & `Decoder` operate oblivious to "SFC" layers. "SFF2" preserves the primary service-path state surrounding these functions, alleviating the need for application parsers to manage service-chain headers.

### 5.4 "Temporal" & "Pose" Packets

Control pathways utilise purposefully minimal payloads. For the "Temporal" chain,

`Encoder` -> `SFF2` ( plain ):

```
"IPv4" + "UDP" + temporal_payload = 20 + 8 + 8 = 36 B
```

`SFF2` -> `SFF1` ( experimental "NSH" ):

```
"IPv4" + "UDP" + nsh_hdr + temporal_payload = 20 + 8 + 8 + 8 = 44 B
```

`SFF1` -> `Camera` ( plain ) reverts to the `36 B` "IPv4" datagram format.

The "Pose" chain accommodates a 12-byte element structure:

```
SFF3 -> SFF2    : 20 + 8 + 8 + 12 = 48 B "IPv4" datagram
SFF2 -> Decoder : 20 + 8 + 12     = 40 B ...
```

Remaining application formats are **pending specification**. They will be documented once the `Decoder` output contract reaches stability.

### 5.5 "Virtio" Queue Size & "DPDK" Rx / Tx Dimensioning

The architecture differentiates the **"virtio-user" queue capacity configured during "vdev" creation** from the **descriptor count requested when initialising each "ethdev" Rx / Tx queue**. These elements represent related controls rather than cumulative buffering strata. For a "virtio-user" "PMD", the "ethdev" queues are supported by "virtqueues", & the descriptor count must align with the capabilities of the "virtio-user" / "vhost" path. Consequently, they must be analysed concurrently.

**"virtio-user" queue size**

The "Environment Abstraction Layer" ( "EAL" ) `queue_size` parameter governs the depth of the "virtqueue" associated with the "vhost-user" transport. A higher value permits more descriptors to remain outstanding between a service function & the "OVS"-"DPDK" "vhost" endpoint before backpressure manifests.

```
Camera                Rx 4096 / Tx 4096
SFF1 Camera-facing    Rx 4096
SFF1 SFF2-facing      Rx 1024
SFF1                  Tx 1024
SFF2 Encoder-facing   Rx 4096
SFF2 Decoder-facing   Rx 4096
SFF2 SFF1/SFF3-facing Rx 1024
SFF2                  Tx 1024
Encoder               Rx 4096 / Tx 4096
```

This asymmetry is deliberate. Links interfacing with compute-intensive end functions are allocated additional elasticity, whereas relay-to-relay interfaces maintain shallower depths, assuming continuous polling loop returns.

These queues buffer finite producer / consumer skew, though the source may still encounter local Tx-ring congestion. Consequently, telemetry isolates active `rte_eth_tx_burst()` execution from wall-clock submission time, explicitly recording zero-accept / resubmission instances.

Chosen depths also interface with shared `mbuf` pools. Each active node requires:

```
NUM_MBUFS       = 16383
MBUF_CACHE_SIZE = 256
```

`SFF2`, for instance, demands a total of:

```
1024 + 4096 + 4096 + 1024 = 10240 Rx descriptors
```

across its four ports. The pool thus maintains sufficient headroom for in-flight packets, burst processing, & per-"lcore" caching, rather than being scaled strictly to the sum of receive rings.

Larger queue capacities do not unilaterally guarantee superior performance. They consume additional memory, can obscure persistent consumer overload, & may elevate the volume of queued traffic before backpressure becomes detectable. Thus, descriptor depth, `queue_size`, core allocation, & cooperative polling must be managed as an **integrated buffering-&-scheduling framework**, held constant across comparative benchmark iterations.

---

## 📷 6. Camera — Scheduling, Warm-Mode, "Temporal" Selection, & Telemetry

### 6.1 Role

The `Camera` functions as the exclusive node originating the volumetric schedule. It ingests the pre-converted "Loot" sequence, allocates source frame IDs, serialises coordinates for network transmission, packetises each point cloud, timestamps the element, & submits "DPDK" bursts to "SFF1". Consequently, its contribution is simultaneously functional & experimental. Every downstream timing quantity is ultimately conditioned by the source cadence, burst structure, cache mode, & residency policy established at this component.

The reference workload is configured as:

```text
K_FRAMES   = 300
TARGET_FPS = 30.0
BURST_SIZE = 32
```

### 6.2 Input Modes

Three distinct cache / storage modes are implemented:

| **Mode** | **Allocation** | **fread()** | **Meaning** |
| -------------------------------- | ----------------- | ----------------- | ----------------------------------------------------------------------------------------------- |
| `CACHE_MODE_BEST`                | Before streaming  | Before streaming  | Cleanest datapath-oriented experiment; frame bytes already reside within the application buffer |
| `CACHE_MODE_MIDDLE`              | Before streaming  | Inside frame loop | Storage-aware experiment; disk / page-cache access remains visible within Camera residency      |
| `CACHE_MODE_WORST`               | Inside frame loop | Inside frame loop | Deliberately pessimistic mode incorporating allocation & file read within the frame path        |

The current source configuration selects:

```text
CACHE_MODE = CACHE_MODE_MIDDLE
WARM_MODE  = WARM_MODE_ENABLED
```

`WARM_MODE_ENABLED` maps & locks source documents prior to the measured sequence, ensuring the timed `fread()` path retains standard file-read semantics while operating across a resident file-backed working set. The resulting `disk_io_ms` should be interpreted as **timed buffered acquisition from a warmed condition**, rather than a direct measurement of cold physical-storage latency.

Results  obtained under varying settings constitute distinct experimental  conditions & must not be conflated within identical performance  claims.

### 6.3 Packetisation

Each selected frame is partitioned into packets containing a maximum of 80 points. The `Camera` assigns:

```text
frame_id
sequence_number
timestamp = t_send_start
temporal_skip
original_points
points_in_packet
```

The `Camera`  timestamp is generated immediately prior to the packet-transmission  loop & propagates unchanged across all packets corresponding to the  same frame.

Coordinates transition from the prepared  little-endian host representation to network-order "IEEE-754" bit  patterns prior to transmission. "RGB" values & the explicit padding  byte remain byte-valued fields.

### 6.4 Isochronous Scheduling & Camera-Side "Temporal" Selection

The source establishes a singular absolute session origin & computes the ideal deadline for frame index `k` as:

```text
T_ideal( k ) = T_0 + k * ( 1 / TARGET_FPS )
```

For the current configuration:

```text
TARGET_FPS = 30
T_base     = 33.333... ms
```

Prior to each source-frame decision, the `Camera` polls for a `temporal_payload`. Should the current factor be `skip`, the frame is selected when:

```text
( frame_id - 1 ) mod skip = 0
```

& the nominal admitted rate resolves to:

```text
FPS = TARGET_FPS / skip
```

Both  selected & skipped frames advance against the identical absolute  source timeline. This mechanism prevents temporal adaptation from  redefining the session clock, thereby preserving meaningful  frame-ID-based scheduling & downstream jitter calculations.

Consequently, the current design omits a separate `PACING_MODE`. Source timing is governed entirely by the absolute target schedule,  whereas local Tx-ring pressure is exposed via retry telemetry rather  than being obscured behind a supplementary pacing heuristic.

### 6.5 Meaning of Tx Backpressure Counters

A zero return from `rte_eth_tx_burst()` indicates the local Tx path accepted zero packets during that specific attempt. The `Camera` retries employing a bounded pause strategy & records:

```text
tx_zero_accepts
tx_partial_accepts
tx_resubmit_calls
tx_resubmitted_packets
mbuf_starvation
```

These quantities must not be classified as "UDP" retransmissions. They transpire **prior to successful local "DPDK" queue acceptance** & thus measure producer / consumer pressure directly at the local packet-I/O boundary.

The  validated run exhibits numerous zero-accept attempts due to the  emission of approximately ten thousand point packets per frame;  nevertheless, every frame remains complete & `mbuf_starvation = 0`.  The counters thereby provide robust evidence that backpressure existed  without translating into application-visible loss during this  experiment.

### 6.6 Camera Telemetry — Complete Semantics

The `Camera` exports **all 23 fields**  delineated below. Their boundaries deliberately segregate logical work,  local "DPDK" queue activity, & the source-residency interval;  consequently, they must not be collapsed into a singular generic  transmission metric.

| **Metric**                | **Unit / Type** | **Exact Definition**                                                                                                              | **Interpretation**                                                                                                                                             |
| ------------------------- | --------------- | --------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `frame_id`                | identifier      | Original source-frame identifier ( `1 ... K_FRAMES` ).                                                                            | Preserves the source timeline even when a subsequent `temporal_skip > 1` induces frame omission.                                                               |
| `status`                  | boolean         | `1` when all declared points are accepted for local "DPDK" transmission & no `mbuf` allocation failure transpires; otherwise `0`. | A temporally non-selected source frame is recorded with `status = 0`, yet it is intentionally absent from the transmitted path rather than classified as lost. |
| `current_skip`            | factor          | Snapshot of the active Camera-side `temporal_skip` sampled prior to the frame-selection decision.                                 | Determines the effective admitted source rate as `TARGET_FPS / current_skip`.                                                                                  |
| `last_control_frame`      | identifier      | Frame identifier conveyed by the most recent admissible "Temporal" control decision received from `SFF1`.                         | Renders controller propagation observable at the source.                                                                                                       |
| `timestamp_start_tx`      | seconds         | `t_send_start / timer_hz`; the Camera timestamp embedded into `cam_hdr.timestamp`.                                                | Shared-host reference utilised by downstream latency metrics.                                                                                                  |
| `tx_points`               | points          | Aggregate of point records belonging to packets successfully accepted by the Camera-facing "DPDK" Tx queue.                       | Evaluated against the source frame population to derive `status`.                                                                                              |
| `tx_packets`              | packets         | Quantity of "DPDK" datagrams successfully accepted for the frame.                                                                 | Approximates `ceil( tx_points / POINTS_PER_PACKET )` within a complete frame.                                                                                  |
| `payload_bytes`           | bytes           | `tx_points * POINT_SIZE_BYTES`.                                                                                                   | Restricts to point bytes only; excludes `cam_hdr` & recurrent network headers.                                                                                 |
| `internal_throughput_mbs` | MB/s            | `( payload_bytes + sizeof( cam_hdr ) ) / 1e6 / send_duration_s`.                                                                  | Decimal logical frame throughput across the Camera submission interval.                                                                                        |
| `logical_bitrate_mbps`    | Mbit/s          | `logical_frame_bytes * 8 * ( TARGET_FPS / current_skip ) / 1e6`.                                                                  | Encompasses point payload alongside one logical `cam_hdr` per frame.                                                                                           |
| `network_bitrate_mbps`    | Mbit/s          | `( payload_bytes + tx_packets * sizeof( main_hdr ) ) * 8 * effective_fps / 1e6`.                                                  | Incorporates recurrent "Eth" / "IPv4" / "UDP" / `cam_hdr` overhead per Camera packet.                                                                          |
| `disk_io_ms`              | ms              | Timed acquisition interval for the designated `CACHE_MODE`.                                                                       | Under `CACHE_MODE_MIDDLE` + `WARM_MODE_ENABLED`, this indicates warmed buffered `fread()` latency, not cold physical-storage latency.                          |
| `serialization_ms`        | ms              | Duration required to translate point coordinates into the explicit on-wire network byte order.                                    | Measures point-record serialisation exclusively.                                                                                                               |
| `tx_duration_ms`          | ms              | `t_send_end - t_send_start`.                                                                                                      | Wall-clock packet-submission span, incorporating local zero-accept retry pauses.                                                                               |
| `active_tx_ms`            | ms              | Aggregate of intervals spent actively executing `rte_eth_tx_burst()` calls.                                                       | Excludes backoff / pause durations between invocations.                                                                                                        |
| `active_process_ms`       | ms              | `disk_io_ms + serialization_ms + tx_duration_ms`.                                                                                 | Reference-compatible active Camera processing boundary.                                                                                                        |
| `total_residency_ms`      | ms              | `t_send_end - t_start_residency`, where residency initiates prior to timed file acquisition.                                      | Comprehensive Camera frame residence for a selected source frame.                                                                                              |
| `node_efficiency_pct`     | %               | `100 * active_process_ms / total_residency_ms`.                                                                                   | Anticipated to approach `100 %` as Camera residence is purposefully bounded around its intrinsic active source path.                                           |
| `tx_zero_accepts`         | count           | Quantity of Tx attempts where `rte_eth_tx_burst()` yields `0`.                                                                    | Local queue backpressure indicator; **not** a "UDP" retransmission sum.                                                                                        |
| `tx_partial_accepts`      | count           | Quantity of Tx attempts accepting fewer packets than requested whilst accepting at least one.                                     | Differentiates partial progression from total zero acceptance.                                                                                                 |
| `tx_resubmit_calls`       | count           | Quantity of subsequent Tx invocations executed following preceding incomplete acceptance.                                         | Enumerates local re-presentation attempts.                                                                                                                     |
| `tx_resubmitted_packets`  | packet-attempts | Aggregate of packet requests introduced by resubmission calls.                                                                    | An identical unsent `mbuf` may contribute repeatedly; thus, this value is characteristically larger than the unique packet population.                         |
| `mbuf_starvation`         | count           | Frame-local `mbuf` allocation failures encountered during packetisation.                                                          | A non-zero incidence may render the frame incomplete even if the transport substrate remains error-free.                                                       |

The principal equations are thus formulated:

```text
logical_frame_bytes     = payload_bytes + sizeof( cam_hdr )
network_frame_bytes     = payload_bytes + tx_packets * sizeof( main_hdr )
effective_fps           = TARGET_FPS / current_skip

internal_throughput_mbs = ( logical_frame_bytes / 1,000,000 ) / tx_duration_s
logical_bitrate_mbps    = logical_frame_bytes * 8 * effective_fps / 1,000,000
network_bitrate_mbps    = network_frame_bytes * 8 * effective_fps / 1,000,000

active_process_ms       = disk_io_ms + serialization_ms + tx_duration_ms
node_efficiency_pct     = 100 * active_process_ms / total_residency_ms
```

A particularly critical distinction concerns Tx resubmission. `tx_resubmitted_packets` operates as an **attempt-volume** counter, rather than a unique packet counter: should the same pending `mbuf`  be presented iteratively while the virtual Tx path rejects it, it  contributes repetitively. This elucidates why the value can legitimately  exceed `tx_packets` by a significant margin without implying duplicated wire traffic.

### 6.7 "End-of-Stream" Behaviour

Following the configured sequence, the `Camera` emits recurrent `END_OF_STREAM` control packets containing the `Camera`  header & devoid of point payload. This redundancy ensures the  terminal condition remains resilient to transient local queue behaviour  within the experimental environment.

The "EOS" marker  constitutes a protocol event, rather than a supplementary source frame,  & is excluded from the 300-frame telemetry table.

---

## 🧠 7. SFF1 — "Geometry-Aware Classifier" ( "GAC" ) & In-Path Aggregation

### 7.1 Role

`SFF1`  operates as the "Geometry-Aware Classifier" ( "GAC" ) within the  current architecture. Its fundamental purpose is to demonstrate that  actionable frame geometry can be derived **while point packets actively traverse the service path**, rather than reconstructing identical statistics initially inside the `Encoder`.

For each valid `Camera` packet, it:

```text
validates "Ethernet" / "IPv4" / "UDP" & Camera metadata
decodes point coordinates from network byte order
updates frame-progressive geometric state
stores "XYZ" within a preallocated frame-local workspace
computes progressive centroid / extent / bounding-box centre
computes exact max_r upon frame completion
replaces the outer network envelope in place
appends experimental "NSH" + geometric context
forwards the original Camera application payload
records frame-level telemetry
```

The  "GAC" is purposefully designed to exceed the capabilities of a mere  forwarding label while remaining substantially narrower than a  full-fledged application processor.

### 7.2 "Temporal" Control Is Relayed, Not Decided, by SFF1

The current `SFF1` iteration does **not** execute source-frame temporal filtering.

"Temporal" adaptation is orchestrated by the `Encoder` & enacted by the `Camera`. `SFF1` functions solely as the terminal service-chain-aware relay along the reverse "Temporal" trajectory:

```text
SFF2 -> SFF1   : "NSH"-encapsulated temporal_payload
SFF1           : validates "SPI" = 200 / "SI" = 255
SFF1           : strips the service-chain envelope
SFF1 -> Camera : plain "UDP" temporal_payload
```

The `current_skip` observed by `SFF1` on the primary data path is, therefore, the factor already adopted by the `Camera` & encapsulated within `cam_hdr`.  It serves telemetry & effective-rate interpretation purposes,  rather than acting as a secondary local frame-drop determinant.

### 7.3 Progressive Geometric Offloading

For an active prefix containing `N` points, `SFF1` maintains:

```text
S_N     = sum_i( p_i )
p_min,N = component-wise min_i( p_i )
p_max,N = component-wise max_i( p_i )
```

& incrementally derives:

```text
C_N = S_N / N
E_N = p_max,N - p_min,N
B_N = ( p_min,N + p_max,N ) / 2
```

These  operations permit updating upon each point's arrival, naturally  aligning with the computational profile of a data-plane-oriented  classifier.

The exact farthest-point radius presents a distinct mathematical dependency:

```text
max_r = max_i || p_i - C_final ||_2
```

Since `C_final` remains indeterminate until the final point is ascertained, exact `max_r`  cannot be resolved universally from the initial packet without either  revisiting preceding points or offloading the computation upstream.  Consequently, the current implementation records only `XYZ` coordinates within a preallocated workspace & executes a secondary radius pass upon frame completion.

This represents a calculated compromise. The computationally expensive application representation is not regenerated inside `SFF1`, packet forwarding persists uninterrupted during the initial pass, & downstream `Encoder` geometry operations are obviated once the final metadata is validated.

### 7.4 In-Place Header Replacement

The incoming `Camera` datagram encompasses:

```text
[ "Ethernet" | "IPv4" | "UDP" | cam_hdr | points ]
```

`SFF1` strips the original network envelope & prepends:

```text
[ "Ethernet" | "IPv4" | "UDP" | nsh_hdr | md2_ctx | geo_agg_hdr ]
```

while preserving:

```text
[ cam_hdr | points ]
```

The resultant packet is structured as:

```text
[ "Ethernet" | "IPv4" | "UDP" | nsh_hdr | md2_ctx | geo_agg_hdr | cam_hdr | points ]
```

This  constitutes the central in-path transformation: service metadata is  appended directly to the packet during forwarding, eschewing the  creation of a disparate frame-level side channel.

### 7.5 Frame-Global Boundary Constraint

Two distinct definitions of a geometric "boundary" exist within the current implementation.

Axis-aligned extrema are packet-progressive:

```text
min_x / max_x
min_y / max_y
min_z / max_z
```

& consequently attain increasing accuracy as packets arrive.

Conversely,  a centroid-dependent radial boundary cannot achieve finality until the  centroid itself is finalised. Similarly, establishing a boundary  contingent upon an arbitrary future user pose necessitates the prior  availability of said pose. No packet format inherently resolves this  mathematical dependency.

Therefore, the project deliberately avoids artificial preprocessing solutions at the `Camera`  or via an offline converter. Such solutions could transmit final  geometry prematurely but would negate the fundamental research objective  of evaluating in-place execution at the service function **on the data path**. The current design upholds this objective, rendering the unavoidable frame-global step explicit.

### 7.6 SFF1 Telemetry — Complete Semantics

`SFF1` exports **36 fields**.  The current iteration treats the node as an active "Geometry-Aware  Classifier" ( "GAC" ); hence, geometry, exact radius, receive span,  active forwarding operations, & overall residence remain  independently verifiable.

| **Metric**                  | **Unit / Type**  | **Exact Definition**                                                                                          | **Interpretation**                                                                                            |
| --------------------------- | ---------------- | ------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------- |
| `frame_id`                  | identifier       | Original Camera frame identifier.                                                                             | Sustains source-relative timing semantics.                                                                    |
| `status`                    | boolean          | `1` when the complete declared point population is received & forwarded; otherwise `0`.                       | Functional integrity indicator for the "GAC" output.                                                          |
| `current_skip`              | factor           | `temporal_skip` duplicated from the Camera header for the current frame.                                      | `SFF1` merely observes this value; it relinquishes source-frame selection.                                    |
| `camera_send_timestamp`     | seconds          | Camera `cam_hdr.timestamp` translated from timer cycles.                                                      | Absolute shared-host source reference.                                                                        |
| `recv_start_timestamp`      | seconds          | Initial accepted packet-arrival timestamp for the frame.                                                      | Denotes node entry.                                                                                           |
| `node_exit_timestamp`       | seconds          | Timestamp assigned to frame completion, typically the final successful Tx activity.                           | Denotes node exit for residency & scheduling analysis.                                                        |
| `original_points`           | points           | Point population stipulated by `cam_hdr.original_points`.                                                     | Reference denominator establishing integrity.                                                                 |
| `rx_points`                 | points           | Total point records validated at `SFF1` ingress.                                                              | Must equate to `original_points` for frame completeness.                                                      |
| `tx_points`                 | points           | Point records correlated with successfully accepted outgoing packets.                                         | Must equate to `rx_points` to yield `status = 1`.                                                             |
| `rx_packets`                | packets          | Validated Camera packets aggregated for the frame.                                                            | Input segmentation metric.                                                                                    |
| `tx_packets`                | packets          | Packets successfully forwarded toward `SFF2`.                                                                 | Output segmentation is anticipated to mirror the Camera packet count.                                         |
| `payload_bytes`             | bytes            | `tx_points * sizeof( point_tx )`.                                                                             | Forwarded point bytes exclusively.                                                                            |
| `data_integrity_pct`        | %                | `100 * rx_points / original_points`.                                                                          | Point-population integrity evaluated at `SFF1` ingress.                                                       |
| `internal_throughput_mbs`   | MB/s             | `( rx_points * 16 + one cam_hdr ) / 1e6 / receive_s`, where `receive_s = last_rx - first_arrival`.            | Pure ingress-span logical throughput, distinct from residency-normalised throughput.                          |
| `logical_bitrate_mbps`      | Mbit/s           | `( tx_point_bytes + one cam_hdr ) * 8 * effective_fps / 1e6`.                                                 | Logical forwarded workload at the corresponding active temporal rate.                                         |
| `network_bitrate_mbps`      | Mbit/s           | `( tx_point_bytes + tx_packets * ( sizeof( main_hdr ) + sizeof( cam_hdr ) ) ) * 8 * effective_fps / 1e6`.     | Incorporates recurrent "Eth" / "IPv4" / "UDP" + experimental "NSH" / context / geometry metadata + `cam_hdr`. |
| `tx_duration_ms`            | ms               | `last_successful_tx - first_successful_tx`.                                                                   | Wall-clock span representing successful egress activity.                                                      |
| `active_tx_ms`              | ms               | Accumulated duration executing `rte_eth_tx_burst()` calls.                                                    | Local Tx execution exclusively.                                                                               |
| `active_process_ms`         | ms               | Accumulated packet-processing, geometry, envelope-rewrite, & flush operations attributed to the frame.        | Numerator utilised for current `node_efficiency_pct`.                                                         |
| `geometry_aggregation_ms`   | ms               | Accumulated first-pass sum / extrema / progressive metadata calculation.                                      | Segregated from the exact `max_r` pass in the present implementation.                                         |
| `max_r_ms`                  | ms               | Exact second-pass farthest-point radius calculation triggered once the final point population is established. | Quantifies the frame-global geometric dependency intentionally offloaded to the "GAC".                        |
| `cycle_ms`                  | ms               | `current_frame_exit - previous_frame_exit` ( initial frame commences from its own initial arrival ).          | Source-cycle occupancy reference.                                                                             |
| `header_wait_ms`            | ms               | `max( 0, cycle_ms - total_residency_ms )`.                                                                    | Inter-frame idle / wait interval external to the current frame residence.                                     |
| `total_residency_ms`        | ms               | `frame_exit - first_arrival`.                                                                                 | Complete `SFF1` residence spanning initial input packet to frame completion / egress.                         |
| `node_efficiency_pct`       | %                | `100 * active_process_ms / total_residency_ms`.                                                               | Fraction of residence directly ascribable to active `SFF1` operations.                                        |
| `camera_to_node_latency_ms` | ms               | `first_sff1_arrival - camera_send_timestamp` synchronised on the shared host timer domain.                    | Local Camera-to-`SFF1` propagation / scheduling interval.                                                     |
| `schedule_delay_ms`         | ms               | `( first_arrival - session_start ) - ( frame_id - first_arrival_frame_id ) / TARGET_FPS`, translated to ms.   | Deviation of processing availability relative to the original isochronous Camera timeline.                    |
| `network_jitter_ms`         | ms               | Absolute disparity between the observed first-arrival spacing & the frame-ID-derived ideal spacing.           | Maintains significance when ensuing temporal gaps render consecutive received IDs non-adjacent.               |
| `eth_errors`                | cumulative count | Snapshot of the process-wide invalid "Ethernet" counter registered upon frame-record initialisation.          | Cumulative diagnostic, **not** a frame-local delta.                                                           |
| `ipv4_errors`               | cumulative count | Snapshot of the process-wide invalid "IPv4" counter.                                                          | Cumulative diagnostic.                                                                                        |
| `udp_errors`                | cumulative count | Snapshot of the process-wide invalid "UDP" counter.                                                           | Cumulative diagnostic.                                                                                        |
| `nsh_errors`                | cumulative count | Snapshot detailing experimental service-header / control-envelope validation failures.                        | Cumulative diagnostic.                                                                                        |
| `tx_zero_accepts`           | count            | Frame-local Tx attempts yielding zero accepted packets.                                                       | Local "DPDK" backpressure indicator.                                                                          |
| `tx_partial_accepts`        | count            | Frame-local Tx invocations resulting in partial acceptance.                                                   | Local "DPDK" backpressure indicator.                                                                          |
| `tx_resubmit_calls`         | count            | Frame-local calls initiated following an incomplete acceptance.                                               | Local resubmission tally.                                                                                     |
| `tx_resubmitted_packets`    | packet-attempts  | Requested packet population aggregated across resubmission calls.                                             | Subject to counting the identical pending packet multiple times.                                              |

The core byte / timing formulations are:

```text
receive_s               = last_rx - first_arrival
logical_rx_frame_bytes  = rx_points * 16 + one cam_hdr
logical_tx_frame_bytes  = tx_points * 16 + one cam_hdr

effective_fps           = TARGET_FPS / current_skip

internal_throughput_mbs = ( logical_rx_frame_bytes / 1,000,000 ) / receive_s
logical_bitrate_mbps    = logical_tx_frame_bytes * 8 * effective_fps / 1,000,000

network_tx_bytes        = tx_points * 16 + tx_packets * ( "Eth" + "IPv4" + "UDP" + nsh_hdr + nsh_md2_ctx_hdr + geo_agg_hdr + cam_hdr )

network_bitrate_mbps    = network_tx_bytes * 8 * effective_fps / 1,000,000

node_efficiency_pct     = 100 * active_process_ms / total_residency_ms
```

`geometry_aggregation_ms` & `max_r_ms`  are deliberately structured to be non-overlapping. The former concludes  subsequent to the progressive sum / extrema update; exclusively  thereafter does the frame-completing branch execute the exact  centroid-dependent radius scan. This separation is imperative for  establishing a defensible measurement of the workload transitioned from `Encoder` to the "GAC".

---

## 🔀 8. SFF2 — Multi-Port "NSH" Proxy & Service-Path Steering

### 8.1 Role & Ports

`SFF2` operates as the central four-port steering hub:

```text
PORT_SFF1    = 0
PORT_ENCODER = 1
PORT_DECODER = 2
PORT_SFF3    = 3
```

Its current functionality transcends a rudimentary forwarder. It constitutes a **stateful experimental "NSH" proxy**  facilitating the coexistence of service-chain-aware functions &  service-chain-unaware applications within an identical trajectory.

The three primary service-path identifiers are:

```text
"Main"     "SPI" = 100
"Temporal" "SPI" = 200
"Pose"     "SPI" = 300
```

### 8.2 Implemented Primary Routing

For `SFF1 -> Encoder`, `SFF2`  ingests the experimental "NSH" geometry envelope, validates the service  state, decrements the "TTL", & instantiates frame-local proxy  state. Subsequently, it strips the incoming service-chain encapsulation  & reconstructs a standard "Ethernet" / "IPv4" / "UDP" packet  destined for the `Encoder`.

The retained application content is formatted as:

```text
[ geo_agg_hdr | cam_hdr | point payload ]
```

Consequently, the `Encoder` processes actionable geometry without necessitating "NSH" parsing.

Upon the return of encoded media from the `Encoder`, `SFF2` identifies the `Encoder`-facing  attachment as the next primary-chain transition, increments the proxy  state, & forwards the unencapsulated application packet towards the  prospective `Decoder`.

This architecture intentionally designates the `Encoder` & `Decoder` attachment domains as **"SFC"-unaware application domains**: service-chain state maintenance is exclusively delegated to the proxy rather than duplicated within application nodes.

### 8.3 "Temporal" & "Pose" Control Paths

The `Encoder` emits an unencapsulated 8-byte `temporal_payload`. `SFF2` discerns this control structure on the `Encoder` port & categorises it as:

```text
"SPI" = 200
"SI"  = 255
```

prior to forwarding it to `SFF1`.

The anticipated pose trajectory operates via an inverse encapsulation logic. A valid "Pose" packet originating from the `SFF3`-facing port applies:

```text
"SPI" = 300
"SI"  = 255
```

`SFF2` strips the service-chain envelope & forwards the 12-byte pose application payload as standard "UDP" to the prospective `Decoder`.

The dual control paths remain purposefully segregated as they address disparate imperatives:

```text
"Temporal" -> Encoder workload regulation -> Camera admission rate
"Pose"     -> User interaction / reconstruction state -> Decoder
```

### 8.4 Burst Ownership

`SFF2`  aggregates outgoing packets into formulated "DPDK" bursts. Each burst  is assigned to the telemetry owner corresponding to the current frame  & route, ensuring successful Tx acceptance is correctly attributed.

This attribution is critical given a single physical `SFF2`  worker processes four logical ports & multiple packet typologies.  In the absence of explicit burst ownership, Tx durations & byte  tallies could be erroneously ascribed during rapid routing transitions.

Therefore,  the implementation flushes an active burst whenever its owner or route  shifts, subsequently recording initial / terminal Tx cycles alongside  active Tx cycles for the respective telemetry entry.

### 8.5 Proxy State & Unaware Service Functions

For each primary frame, `SFF2` retains a minimal proxy context comprising:

```text
frame_id
"TTL"
"SI"
valid state
```

The state is registered when a legitimate `SFF1` primary packet permeates the proxy boundary. While the packet remains decapsulated for an unaware function, `SFF2` upholds responsibility for managing the logical service-index transition.

This architecture offers two distinct advantages for the experiment.

Firstly, the `Encoder`  concentrates exclusively on point processing, "CUDA" acceleration,  & "codec" mechanics without assuming the burden of a secondary  service-chain parser. Secondly, this methodology extends to the  prospective `Decoder`,  facilitating comparative analysis of application processing without  demanding service-chain logic integration within each application node.

The  concession is explicit statefulness situated within the forwarder.  Frame identity must endure validly & consistently to enable the  proxy to increment or re-impose service state accurately.

The previously documented repository detailing a `RED`-like / `AQM` heuristic is obsolete within this snapshot. The present `SFF2` source omits such congestion controllers; workload adaptation has been entirely relocated to the `Encoder`-driven "Temporal" path.

### 8.6 Route-Specific Payload Semantics

Route 0 ( `SFF1 -> Encoder` ) conveys point data:

```text
logical payload -> point bytes + one cam_hdr
application context visible to Encoder -> geo_agg_hdr
```

Route 1 ( `Encoder -> Decoder` ) transports encoded media:

```text
logical payload -> "MPEG-TS" bytes + one cam_hdr + one enc_hdr
```

Route 2 ( `Decoder -> SFF3`  ) remains intentionally devoid of a stable application-byte formula.  The base proxy branch is capable of re-imposing service metadata upon  receiving a packet from the future `Decoder` possessing a valid frame identifier, yet the `Decoder` payload schema, completion semantics, & `SFF3` contract lack finality.

This differentiation is crucial: **proxy scaffolding is established for Route 2, yet validated Route-2 telemetry remains absent**.

### 8.7 SFF2 Telemetry — Complete Semantics

`SFF2`  implements a uniform 36-column telemetry schema across its route  arrays. Within the current validated snapshot, quantitative metrics hold  significance exclusively for Route 0 ( `SFF1 -> Encoder` ) & Route 1 ( `Encoder -> Decoder` ). Route 2 maintains the proxy / forwarding framework but purposefully lacks committed application-format accounting.

| **Metric**                  | **Unit / Type**  | **Exact Definition**                                                                                                                | **Interpretation**                                                                                             |
| --------------------------- | ---------------- | ----------------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- |
| `frame_id`                  | identifier       | Source frame identifier derived from `cam_hdr`.                                                                                     | Orchestrates the binding between route telemetry & per-frame proxy context.                                    |
| `status`                    | boolean          | Route  0: complete point reception + precise point forwarding. Route 1:  non-empty media reception + precise media-byte forwarding. | Route-specific integrity flag.                                                                                 |
| `current_skip`              | factor           | "Temporal" skip parameter conveyed within `cam_hdr`.                                                                                | Dictates `effective_fps = TARGET_FPS / current_skip`.                                                          |
| `camera_send_timestamp`     | seconds          | Original Camera timestamp preserved throughout the "Main" trajectory.                                                               | Facilitates cross-node referencing.                                                                            |
| `recv_start_timestamp`      | seconds          | Initial packet arrival logged at the specific `SFF2` route.                                                                         | Denotes node-entry.                                                                                            |
| `node_exit_timestamp`       | seconds          | Terminal route timestamp, generally indicating the last successful Tx / last relevant operation.                                    | Denotes node-exit.                                                                                             |
| `original_points`           | points           | Original point count extracted from `cam_hdr`.                                                                                      | Retains significance on both routes as source metadata; Route 1 strictly handles compressed media.             |
| `rx_points`                 | points           | Ingress point count for Route 0.                                                                                                    | Architecturally expected to be zero on Route 1.                                                                |
| `tx_points`                 | points           | Successfully forwarded point count for Route 0.                                                                                     | Architecturally expected to be zero on Route 1.                                                                |
| `rx_media_bytes`            | bytes            | Compressed-media bytes received on Route 1 subsequent to `cam_hdr + enc_hdr`.                                                       | Anticipated to be zero on Route 0.                                                                             |
| `tx_media_bytes`            | bytes            | Compressed-media bytes on Route 1 correlated with successful Tx packets.                                                            | Anticipated to match `rx_media_bytes` for `status = 1`.                                                        |
| `rx_packets`                | packets          | Validated packets upon route ingress.                                                                                               | Point datagrams on Route 0; compressed-media datagrams on Route 1.                                             |
| `tx_packets`                | packets          | Packets successfully accepted during route egress.                                                                                  | Utilised for calculating recurrent wire-envelope overhead.                                                     |
| `payload_bytes`             | bytes            | Route 0: `rx_points * 16`. Route 1: `rx_media_bytes`.                                                                               | Represents route-native application payload.                                                                   |
| `data_integrity_pct`        | %                | Route 0: `100 * rx_points / original_points`. Route 1: `100 * tx_media_bytes / rx_media_bytes`.                                     | Point-integrity versus byte-preservation semantics are deliberately route-specific.                            |
| `internal_throughput_mbs`   | MB/s             | `logical_rx_frame_bytes / 1e6 / receive_s`, where `receive_s = last_rx - first_arrival`.                                            | Designates ingress logical throughput.                                                                         |
| `logical_bitrate_mbps`      | Mbit/s           | `logical_tx_frame_bytes * 8 * effective_fps / 1e6`.                                                                                 | Encompasses application bytes + singular instance of frame metadata.                                           |
| `network_bitrate_mbps`      | Mbit/s           | Route-specific egress bytes including the repeated network envelope facing the Encoder or Decoder.                                  | Illustrates current "DPDK" datagram construction, excluding physical preamble / "FCS".                         |
| `tx_duration_ms`            | ms               | `last_successful_tx - first_successful_tx`.                                                                                         | Spans the route egress interval.                                                                               |
| `active_tx_ms`              | ms               | Accumulated duration engaged in route Tx-burst operations.                                                                          | Represents local Tx execution.                                                                                 |
| `active_process_ms`         | ms               | Accumulated work across route classification, proxy processing, header rewriting & Tx.                                              | Serves as the numerator for `node_efficiency_pct`.                                                             |
| `cycle_ms`                  | ms               | `route_frame_exit - previous_route_frame_exit`.                                                                                     | Denotes the per-route source cycle.                                                                            |
| `header_wait_ms`            | ms               | `max( 0, cycle_ms - total_residency_ms )`.                                                                                          | Identifies the inter-frame idle interval specific to that route.                                               |
| `total_residency_ms`        | ms               | `route_frame_exit - first_route_arrival`.                                                                                           | Spans complete frame residence within the chosen `SFF2` route.                                                 |
| `node_efficiency_pct`       | %                | `100 * active_process_ms / total_residency_ms`.                                                                                     | Represents the active steering / proxy quotient of overall residence.                                          |
| `camera_to_node_latency_ms` | ms               | `first_route_arrival - camera_send_timestamp`.                                                                                      | Absolute shared-host Camera-relative latency; distinct from the Encoder field as it omits baseline-correction. |
| `schedule_delay_ms`         | ms               | First-arrival variation from `( frame_id - first_arrival_frame_id ) / TARGET_FPS`.                                                  | Sustains the source timeline irrespective of temporal frame-ID discontinuities.                                |
| `network_jitter_ms`         | ms               | Absolute recorded inter-arrival error against the frame-ID-derived anticipated interval.                                            | Highlights route-level timing dispersion.                                                                      |
| `eth_errors`                | cumulative count | Snapshot capturing process-wide "Ethernet" validation failures.                                                                     | Functions as a cumulative diagnostic.                                                                          |
| `ipv4_errors`               | cumulative count | Snapshot capturing process-wide "IPv4" validation failures.                                                                         | Functions as a cumulative diagnostic.                                                                          |
| `udp_errors`                | cumulative count | Snapshot capturing process-wide "UDP" validation failures.                                                                          | Functions as a cumulative diagnostic.                                                                          |
| `nsh_errors`                | cumulative count | Snapshot detailing proxy / experimental "NSH" validation-state anomalies.                                                           | Cumulative diagnostic; may also indicate fragmented per-frame proxy states.                                    |
| `tx_zero_accepts`           | count            | Route-local Tx attempts concluding with zero acceptance.                                                                            | Signifies local queue pressure.                                                                                |
| `tx_partial_accepts`        | count            | Route-local partial Tx acceptances.                                                                                                 | Signifies local queue pressure.                                                                                |
| `tx_resubmit_calls`         | count            | Route-local resubmission invocations succeeding an incomplete Tx attempt.                                                           | Does not indicate a transport retransmission.                                                                  |
| `tx_resubmitted_packets`    | packet-attempts  | Packet requests generated through those resubmission invocations.                                                                   | May repeatedly count a single pending packet across multiple attempts.                                         |

The route-specific formulations operate as follows:

```text
Route 0 ( SFF1 -> Encoder )

logical_rx_frame_bytes = rx_points * 16 + one cam_hdr
logical_tx_frame_bytes = tx_points * 16 + one cam_hdr

network_tx_bytes       = tx_points * 16 + tx_packets * ( "Eth" + "IPv4" + "UDP" + geo_agg_hdr + cam_hdr )

integrity              = 100 * rx_points / original_points

Route 1 ( Encoder -> Decoder )

logical_rx_frame_bytes = rx_media_bytes + one cam_hdr + one enc_hdr
logical_tx_frame_bytes = tx_media_bytes + one cam_hdr + one enc_hdr

network_tx_bytes       = tx_media_bytes + tx_packets * ( "Eth" + "IPv4" + "UDP" + cam_hdr + enc_hdr )

integrity              = 100 * tx_media_bytes / rx_media_bytes
```

Consistent across both validated routes:

```text
receive_s               = last_rx - first_arrival
internal_throughput_mbs = ( logical_rx_frame_bytes / 1,000,000 ) / receive_s
node_efficiency_pct     = 100 * active_process_ms / total_residency_ms
```

The metrics `eth_errors`, `ipv4_errors`, `udp_errors`, & `nsh_errors`  represent snapshots of process-wide cumulative diagnostics, whereas Tx  resubmission parameters are exclusively frame / route local. This  differentiation must be strictly adhered to when aggregating CSV rows:  summarising cumulative error columns would inadvertently result in the  double-counting of historical failures.

---

## ⚙️ 9. Encoder — Hybrid Frame Processing, "CUDA", "Temporal" Control, & "H.265"

### 9.1 Role

The `Encoder` is deliberately engineered to remain "NSH"-unaware. It processes  standard "UDP" packets where the application payload initiates with the  geometric context imparted by the `SFF2` proxy:

```text
[ geo_agg_hdr | cam_hdr | points ]
```

Its functional paradigm integrates two distinct execution models.

Upon  packet ingress, it functions in a data-plane-oriented capacity:  coordinates are decoded & written instantaneously into deterministic  frame offsets, packet-completion status is incremented continuously,  & the optimal geometry snapshot is preserved. Simultaneously, during  intensive "CPU" / "GPU" cycles, the implementation sustains network  reception through cooperative polling.

Conversely, at  the projection phase, a stringent frame-level readiness constraint is  enforced, given that the exact active point set & definitive  geometry must be fully resolved. Consequently, the `Encoder` constitutes a **hybrid frame-aware entity**,  uniquely positioned between a traditional passive application awaiting  complete objects & an atomic packet-local data-plane operation.

### 9.2 Frame Assembly

The frame buffer orchestrates tracking for:

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

Packet  sequence numbers definitively govern destination offsets. Therefore,  point conversion executes progressively as packets arrive, negating the  necessity for an independent post-reception frame conversion traversal.

A  standard frame is deemed processable exclusively when all anticipated  packets & points are successfully aggregated. Upon an "EOS"  condition, a fragmentary final frame may undergo compaction &  processing; however, the validated 300-frame test suite comprises only  complete frames.

### 9.3 Selecting the Most Complete Geometry Snapshot

`SFF1` continuously broadcasts progressive geometry. Consequently, the `Encoder` isolates the snapshot possessing the maximum valid `active_point_count` identified for that specific frame.

For a finalised frame, `geometry_from_sff1()` authorises the offloaded metrics strictly when:

```text
metadata_active_points == active_point_count
```

& all decoded geometry components demonstrate finiteness & internal consistency.

This mechanism precludes an early progressive packet from being erroneously interpreted as the definitive geometric description.

### 9.4 Frame Readiness & Incomplete Frames

The  conventional processing trajectory demands absolute point completeness  preceding projection, as the six-view representation & "codec" input  inherently rely on frame-level aggregates.

However, this barrier does not imply that antecedent tasks are postponed. Prior to achieving readiness, the `Encoder` has proactively:

```text
validated packet headers
converted point coordinates
placed points at deterministic offsets
updated receive counters
tracked geometric metadata progression
recorded arrival timing
```

The  residual barrier is strictly confined to operations yielding inherently frame-level results, avoiding passive stagnation across the  computational pipeline.

### 9.5 Division of Geometric Work & Selectable Offload

The build exposes the following configurations:

```text
OFFLOAD_MODE_DISABLED = 0
OFFLOAD_MODE_ENABLED  = 1
OFFLOAD_MODE          = OFFLOAD_MODE_ENABLED ( actual run )
```

When offload is activated & complete "GAC" metadata is validated, the `Encoder` secures:

```text
centroid
extent
raw bounding-box centre
max_r
```

without executing redundant local frame scans. Under conditions of complete offloaded frames, telemetry dictates:

```text
geometry_aggregation_ms = 0
max_r_ms                 = 0
```

Should metadata prove unavailable, inconsistent, deactivated, or partial, `geometry_recompute_local()`  provides a robust functional fallback. It reconstructs sums / extrema,  derives the centroid / extent / box centre, & executes the radius  pass across the active point set while persistently polling "DPDK".

The ultimate object scale, instituted prior to projection, hinges upon the radius target:

```text
target_radius = CAMERA_DISTANCE * 0.2
final_scale   = target_radius / max_r
```

contingent upon implementation-level validity verifications.

This selectable trajectory holds profound experimental significance: it facilitates a highly controlled `OFFLOAD_MODE_ENABLED` versus `OFFLOAD_MODE_DISABLED` comparative analysis without altering the underlying packet-processing architecture.

### 9.6 Workload-Driven "Temporal" Controller

"Temporal" adaptation is strategically centralised at the `Encoder`, as the primary experimental bottleneck resides at the processing node rather than a superficial user-side quality selector.

The controller extracts a service-time sample `T_n` & formulates an exponentially weighted moving average:

```text
T_base            = 1000 / TARGET_FPS
T_budget( skip )  = skip * T_base
E_n               = alpha * T_n + ( 1 - alpha ) * E_( n - 1 )
workload_ratio    = E_n / T_budget( active_skip )
```

Current operational parameters include:

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

The  controller necessitates an initial stable start-up window. Once primed,  an overload condition is triggered when any of the ensuing predicates  evaluate as true:

```text
workload_ratio >= 0.90
wait_raw_queue_ms > 0.25 * active_budget_ms
frame_backlog >= 2
```

Following two consecutive overload recognitions, the requested skip increments by a factor of one, up to the defined `MAX_SKIP`.

Recovery protocols are purposely more conservative. For `active_skip > 1`, the controller assesses whether the current "EWMA" would successfully conform within `75 %` of the budget designated at `skip - 1`, provided the raw-queue wait remains under `10 %`  of the active budget & the frame backlog is sustained at zero. Nine  consecutive recoverable observations must be registered before a  reduction in the skip factor is authorised.

If the `Camera` fails to mirror a requested adjustment, the `Encoder` reiterates the control decision every three observations, mitigating runaway request escalations.

Logged events manifest as:

```text
"WARMUP"
"IDLE"
"SKIP+1"
"SKIP-1"
"RETRY"
"INVALID"
```

The referenced validation sequence records five initial `WARMUP` events followed by 295 `IDLE` events. The parameter `current_skip` remains strictly `1` across all 300 frames, indicating the workload ratio never breaches the overload threshold & `frame_backlog` persistently stays at zero.

### 9.7 "CUDA" Memory Strategy

The  projection pipeline proactively preallocates persistent device buffers,  a designated "CUDA" stream, & precise timing events. Host "I420"  output slots are similarly allocated at inception & formally  registered with "CUDA".

Current buffering frameworks incorporate:

```text
H2D_CHUNK_POINTS = 65536
YUV_BUFFER_COUNT = 3
```

The  point array is transferred asynchronously via designated chunks. Prior  to each chunk transfer, the "CUDA" path asserts the capacity to invoke  the `Encoder`'s "DPDK" polling callback. Following kernel initiation & subsequent copy-back, the worker continually polls while `cudaStreamQuery()` denotes the stream as incomplete.

This  configuration strictly curtails the duration wherein "GPU" submission  obstructs packet reception & circumvents repetitive device  allocations along the measured pathway.

### 9.8 "CUDA" Projection Stages

The  static pose implemented within the current experiment permits the  pipeline to bypass a distinct point-wise transformed-bounding-box  reduction.

The raw bounding-box centre is mathematically transformed directly around the centroid & final scale:

```text
B'_x = ( B_x - C_x ) * final_scale
B'_y = ( B_y - C_y ) * final_scale
B'_z = ( B_z - C_z ) * final_scale + CAMERA_DISTANCE
```

These scaled extents subsequently define a comprehensive global orthographic scale. Expressed programmatically:

```text
global_scale = 1.10 * max( extent'_x / WIDTH, extent'_y / HEIGHT, extent'_z / WIDTH )
```

The multiplier `1.10` ensures adequate projection margin.

The ensuing stages are structured as:

```text
asynchronous "H2D" transfer
fused point projection / colour conversion / z-buffer update
atlas packing
asynchronous "D2H" copy
```

Support  for dynamic poses will eventually necessitate a reassessment of this  optimisation once pose-dependent variables are actively introduced  downstream.

### 9.9 Six-View G-Buffer

The integrated "CUDA" kernel executes the following per point:

```text
object-centred scaling
normalised-coordinate construction
"BT.601" "RGB" -> "YUV" conversion
six orthographic face projections
depth quantisation
atomic z-buffer visibility arbitration
geometry / occupancy / texture updates
```

The directive `atomicMax()`  mitigates visibility conflicts within the integer depth buffer. This  paradigm localises intensive point-parallel computations upon the "GPU",  leaving only streamlined geometric control formulations resident on the  host.

### 9.10 Atlas Geometry

The foundational projected view operates on:

```text
WIDTH  = 640
HEIGHT = 480
```

Faces receive padding to:

```text
FACE_W_PADDED = 640
FACE_H_PADDED = 512
```

& are structurally aligned in a four-by-three cross formation:

```text
CROSS_W = 2560
CROSS_H = 1536
```

Three identical, vertically stacked crosses represent:

```text
"Geometry"
"Texture"
"Occupancy"
```

yielding an encoded "I420" frame dimension of:

```text
ENCODER_W = 2560
ENCODER_H = 4608
```

The corresponding uncompressed "I420" buffer demands:

```text
TOTAL_YUV_SIZE = 17,694,720 B
```

per actively submitted frame.

### 9.11 "GPU" Timing Probes

The `Encoder` logs the following metrics:

```text
gpu_transfer_ms
gpu_kernel_ms
gpu_packing_ms
gpu_copyback_ms
host_overhead_ms
projection_ms
```

"CUDA" event chronologies isolate strictly asynchronous "GPU" operations, whereas `projection_ms`  captures the full host-visible interval enveloping the projection call.  Given that cooperative "DPDK" polling proceeds while the stream  maintains an incomplete status, `host_overhead_ms` signifies residual host-visible time rather than acting as a definitive arithmetic-"CPU" kernel metric.

### 9.12 Persistent "FFmpeg" / "NVENC" Process & Pre-Roll

A  singular, persistent "FFmpeg" process initiates concurrently with the  experiment. Contemporary rate-control & latency-oriented parameters  specify:

```text
"codec"          = hevc_nvenc
preset           = p2
tune             = ull
rate control     = cbr
target bitrate   = 10M
buffer size      = 20M
"GOP"            = 15
B frames         = 0
lookahead        = 0
delay            = 0
zero latency     = 1
flush packets    = 1
```

Preceding the measurement of application frames, the `Encoder` propels one "GOP" consisting of private, unrecorded pre-roll frames:

```text
FRAMES = 15
```

This pre-roll sequence traverses the standard writer & parser conduits, propelling the persistent "codec", muxer, & attribution logic into  an operational, warmed state. Crucially, pre-roll frames bypass application network transmission entirely & remain omitted from the 300-row `Encoder` telemetry.

A  dedicated writer thread & three designated "I420" slots uncouple  the projection phase from blocking interactions with "FFmpeg" stdin. In  instances where all slots are committed, the `Encoder` perpetually services network / "codec" activities while awaiting slot emancipation.

### 9.13 Why "MPEG-TS" / "PES" Parsing Is Necessary

Pipe reads sourced from "FFmpeg" inherently fail to preserve distinct video-frame boundaries. Thus, the `Encoder`  is obliged to reconstruct strictly structured 188-byte "MPEG-TS"  packets from variable read sizes & identify precise video "PES"  demarcations.

The initial identified video-"PES"  boundary corresponds directly to the oldest frame assigned to "FFmpeg".  This enables the derivation of:

```text
encode_h265_ms = first_associated_PES - ffmpeg_input_start
```

This metric consolidates asynchronous "codec", scheduling, muxing, & pipe-delivery contingencies. It is emphatically **not** promoted as an isolated "NVENC" hardware-kernel execution interval.

### 9.14 Encoder Output Packetisation

The `Encoder` systematically aggregates seven complete TS packets:

```text
7 * 188 B = 1316 B
```

& affixes the subsequent headers:

```text
cam_hdr
enc_hdr
```

before constructing an unencapsulated "UDP" packet destined for the `SFF2` proxy.

Notably, the `Encoder` issues no service-chain headers. Primary-path "NSH" state management persists unambiguously as an `SFF2` prerogative.

### 9.15 Encoder Telemetry — Complete Semantics

The `Encoder` generates **54 fields**,  cementing its status as the most exhaustively instrumented node within  the established chain. This schema deliberately separates raw ingestion,  local geometry manipulation, "CUDA" wall / device durations, writer  mechanics, workload regulation, "codec" materialisation, compressed  egress, & partial `Camera`-relative delays.

| **Metric**                  | **Unit / Type**     | **Exact Definition**                                                                                                                                         | **Interpretation**                                                                                                                                                                       |
| --------------------------- | ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `frame_id`                  | identifier          | Original Camera frame identifier.                                                                                                                            | Binds raw input, geometry, controller state, "codec" attribution, & compressed output.                                                                                                   |
| `status`                    | boolean             | `1` designates a valid frame record; drops to `0` upon geometric / enqueue anomalies.                                                                        | Denotes functional validity.                                                                                                                                                             |
| `current_skip`              | factor              | "Temporal" factor transmitted by the Camera frame instigating this Encoder record.                                                                           | The workload controller may stipulate divergent values for succeeding source frames.                                                                                                     |
| `event`                     | state               | Adopts one of: "WARMUP", "IDLE", "SKIP+1", "SKIP-1", "RETRY", or "INVALID".                                                                                  | Renders controller state transitions definitively observable.                                                                                                                            |
| `yaw`                       | degrees / reference | Extant upstream yaw value; precisely `0.0` within the validated scenario.                                                                                    | Remains static until the forthcoming "Pose" path integrates with Decoder-side reconstruction.                                                                                            |
| `pitch`                     | degrees / reference | Extant upstream pitch value; precisely `0.0` within the validated scenario.                                                                                  | Represents a static reference.                                                                                                                                                           |
| `zoom`                      | factor              | Extant upstream zoom value; precisely `1.0` within the validated scenario.                                                                                   | Represents a static reference.                                                                                                                                                           |
| `camera_send_timestamp`     | seconds             | Camera frame Tx-start timestamp disseminated via `cam_hdr`.                                                                                                  | Establishes the core source timing anchor.                                                                                                                                               |
| `recv_start_timestamp`      | seconds             | Primary Encoder packet-arrival point assigned to the frame.                                                                                                  | Denotes the initial Encoder node-entry anchor.                                                                                                                                           |
| `node_exit_timestamp`       | seconds             | Conclusive compressed "DPDK" egress timestamp correlated with the frame.                                                                                     | Denotes the definitive Encoder node-exit anchor.                                                                                                                                         |
| `clock_offset_ms`           | ms                  | Derived from the initial processed frame: `( first_encoder_arrival - first_camera_tx ) * 1000`; reapplied as a baseline.                                     | Eliminates the foundational shared-host path discrepancy from Encoder Camera-relative latency assessments.                                                                               |
| `original_points`           | points              | Aggregate point tally declared by the Camera.                                                                                                                | Establishes the baseline input completeness denominator.                                                                                                                                 |
| `rx_points`                 | points              | Verified unique point records collated by sequence.                                                                                                          | Benchmarked directly against `original_points`.                                                                                                                                          |
| `tx_points`                 | points represented  | Configured to mirror the specific point population articulated by the encoded frame.                                                                         | Expressly does **not** signify that raw point records traverse Route 1.                                                                                                                  |
| `rx_packets`                | packets             | Confirmed point packets advancing the frame.                                                                                                                 | Illustrates input segmentation granularity.                                                                                                                                              |
| `tx_packets`                | packets             | Compressed-media "UDP" datagrams admitted by the Encoder Tx conduit.                                                                                         | Illustrates output segmentation granularity.                                                                                                                                             |
| `payload_bytes`             | bytes               | Summarised calculation of input point bytes solely.                                                                                                          | Extrudes incoming `geo_agg_hdr` / `cam_hdr` parameters.                                                                                                                                  |
| `data_integrity_pct`        | %                   | `100 * rx_points / original_points`.                                                                                                                         | Defines raw point-population fidelity prior to projection operations.                                                                                                                    |
| `internal_throughput_mbs`   | MB/s                | `( payload_bytes + one cam_hdr ) / 1e6 / ( last_arrival - first_arrival )`.                                                                                  | Formulates logical raw-input velocity over the absolute receive span.                                                                                                                    |
| `logical_bitrate_mbps`      | Mbit/s              | `( mpeg_bytes_generated + one cam_hdr + one enc_hdr ) * 8 * effective_fps / 1e6`.                                                                            | Calculates  compressed logical-output volume; distinctly separate from the  raw-input volume prevalent in the reference application.                                                     |
| `network_bitrate_mbps`      | Mbit/s              | `( mpeg_bytes_generated + tx_packets * ( "Eth" + IPv4 + "UDP" + cam_hdr + enc_hdr ) ) * 8 * effective_fps / 1e6`.                                            | Portrays compressed egress factoring in repetitive packet header footprints.                                                                                                             |
| `conversion_ms`             | ms                  | Accumulated per-packet transformation from network-to-host coordinates & alignment within the frame buffer.                                                  | Materialises incrementally concurrent with packet ingress.                                                                                                                               |
| `geometry_aggregation_ms`   | ms                  | Time expended on local centroid / extrema / bounding-box calculations when `SFF1` offload proves unviable.                                                   | Yields `0` across complete offloaded frames under representative evaluations.                                                                                                            |
| `max_r_ms`                  | ms                  | Local rigorous farthest-point radius sequence when demanded.                                                                                                 | Yields `0` across complete offloaded frames; however, partial "EOS" recoveries may necessitate recalculation.                                                                            |
| `projection_ms`             | ms                  | Wall-clock  span bridging projection submission initiation to culmination,  inclusive of cooperative "DPDK" polling amidst asynchronous "CUDA"  progression. | Conceptually broader than the arithmetic sum of isolated device event durations.                                                                                                         |
| `tx_duration_ms`            | ms                  | Wall-clock span of the designated writer actively feeding a unified "I420" frame into the persistent "FFmpeg" stdin.                                         | Incorporates blocking pipe intricacies; operates as the reference-compatible render / "codec" handover component.                                                                        |
| `active_process_ms`         | ms                  | Within the extant implementation: `conversion + geometry_aggregation + max_r + projection + tx_duration`.                                                    | Retained uniformly identical to `total_processing_ms` ensuring reference-compatible node-efficiency parameters.                                                                          |
| `total_processing_ms`       | ms                  | Mirrors the precise arithmetic sum composing `active_process_ms`.                                                                                            | Expressly prohibits recounting the asynchronous `encode_h265_ms` duration.                                                                                                               |
| `total_residency_ms`        | ms                  | `last_encoded_dpdk_egress - first_point_arrival`.                                                                                                            | Spans frame assemblage, queueing, projection, writer / "codec" interfacing, & compressed egress phases.                                                                                  |
| `node_efficiency_pct`       | %                   | `100 * active_process_ms / total_residency_ms`.                                                                                                              | Determines the processing quotient embedded within the overarching Encoder residence.                                                                                                    |
| `gpu_transfer_ms`           | ms                  | "CUDA" event span stretching from projection initiation across asynchronous "H2D" point relocation.                                                          | Constitutes the device-timeline transfer stage.                                                                                                                                          |
| `gpu_kernel_ms`             | ms                  | "CUDA" event span extending from the conclusion of "H2D" transfer throughout the fused 6-view "G-Buffer" projection.                                         | Absorbs object-centric transformations, "BT.601" calibrations, projection mechanisms, & atomic depth resolution.                                                                         |
| `gpu_packing_ms`            | ms                  | "CUDA" event span dedicated to atlas / "I420" alignment post-projection.                                                                                     | Constitutes the device packaging stage.                                                                                                                                                  |
| `gpu_copyback_ms`           | ms                  | "CUDA" event span allocated for the concluding device-to-host "I420" retrieval.                                                                              | Constitutes the device-timeline "D2H" component.                                                                                                                                         |
| `host_overhead_ms`          | ms                  | `max( 0, projection_ms - gpu_transfer_ms - gpu_kernel_ms - gpu_packing_ms - gpu_copyback_ms )`.                                                              | Represents residual wall time inclusive of host orchestration & cooperative polling; decisively **not** a sterile "CPU"-compute indicator.                                               |
| `camera_to_node_latency_ms` | ms                  | `first_arrival - camera_tx - clock_offset`.                                                                                                                  | Baseline-calibrated Camera-to-Encoder-arrival deviation; notably, the primary frame operates intentionally proximal to `0`.                                                              |
| `end_to_end_latency_ms`     | ms                  | `node_exit - camera_send_timestamp - clock_offset`.                                                                                                          | Designates a partial Camera-to-Encoder-compressed-egress delay, decisively **not** the definitive Camera-to-User E2E metric.                                                             |
| `schedule_delay_ms`         | ms                  | `( service_start - first_encoder_session_arrival ) - frame_offset / TARGET_FPS`.                                                                             | Records service-start deviation divergent from the prescribed source rhythm.                                                                                                             |
| `network_jitter_ms`         | ms                  | Absolute first-arrival spacing discrepancy relative to `( frame_id_gap / TARGET_FPS )`.                                                                      | Articulates ID-gap-cognisant timing variability.                                                                                                                                         |
| `wait_raw_queue_ms`         | ms                  | `service_start - frame_ready`; assuming the aggregate frame remains incomplete at finalisation, the fallback pivot is `last_arrival`.                        | Quantifies raw-frame detention post-readiness preceding the inception of Encoder service.                                                                                                |
| `wait_render_queue_ms`      | ms                  | `ffmpeg_write_start - projection_end`.                                                                                                                       | Encapsulates  purely the post-projection writer-queue detention; pre-projection  "I420" slot procurement remains deliberately exempted.                                                  |
| `workload_ewma_ms`          | ms                  | `E_n = alpha * service_n + ( 1 - alpha ) * E_(n-1)` incorporating `alpha = 0.25`; resets to the prevailing sample upon warm-up activation.                   | Represents the smoothed Encoder worker-service signal fuelling temporal adaptation frameworks.                                                                                           |
| `workload_ratio`            | ratio               | `workload_ewma_ms / ( current_skip * 1000 / TARGET_FPS )`.                                                                                                   | Delineates controller load proportionate to the operative temporal budget.                                                                                                               |
| `frame_backlog`             | frames              | Quantity of supplementary frame buffers queued following the election of the prevailing frame (`frame_buffers.size() - 1`).                                  | Functions as a direct raw-input backlog catalyst; an overload state materialises upon reaching a value of `2`.                                                                           |
| `codec_backlog`             | frames              | `max( writer_pending_frames(), mpeg_frame_queue.size() )`.                                                                                                   | Provides a diagnostic of "codec" / writer strain. Within the extant controller topology, it remains **observed but not directly integrated** into the overload / recovery boolean logic. |
| `encode_h265_ms`            | ms                  | Elapsed  chronology traversing the "FFmpeg" writer inception for a genuine frame  to the realisation of the premier attributed video-"PES" boundary.         | Designates the primary verifiable encoded-output lag, distinct from an isolated "NVENC" kernel execution duration.                                                                       |
| `mpeg_bytes_generated`      | bytes               | "MPEG-TS" data attributed by the parser to the fundamental source frame preceding network-packet allocation.                                                 | Assuming a zero `mbuf_starvation` state, this value precisely matches the media-byte payload acquired by `SFF2` Route 1 in the representative execution.                                 |
| `ffmpeg_write_calls`        | count               | Quantity of `write()` system commands commissioned to dispatch the frame to the "FFmpeg" stdin.                                                              | The extant run persistently records one command per genuine frame.                                                                                                                       |
| `ffmpeg_write_eagain`       | count               | Quantity of defensive `EAGAIN` / `EWOULDBLOCK` resolutions intercepted by the writer.                                                                        | Anticipated to preserve a zero value given the existing blocking input pipe configuration.                                                                                               |
| `tx_zero_accepts`           | count               | Compressed-output Tx commands eliciting zero accepted packets.                                                                                               | Identifies local "DPDK" backpressure situations.                                                                                                                                         |
| `tx_partial_accepts`        | count               | Compressed-output Tx commands yielding fractional acceptance.                                                                                                | Identifies local "DPDK" backpressure situations.                                                                                                                                         |
| `tx_resubmit_calls`         | count               | Compressed-output resubmission commands following an incomplete acceptance.                                                                                  | Categorically not a "UDP" retransmission parameter.                                                                                                                                      |
| `tx_resubmitted_packets`    | packet-attempts     | Packet requests generated via said resubmission commands.                                                                                                    | Retains the capability to count a single pending packet redundantly.                                                                                                                     |
| `mbuf_starvation`           | count               | Compressed-output `mbuf` allocation / linkage failures.                                                                                                      | Should this possess a non-zero value, `mpeg_bytes_generated` may inherently eclipse the volume of data factually transmitted.                                                            |

The present processing & residency formulae dictate:

```text
total_processing_ms = conversion_ms + geometry_aggregation_ms + max_r_ms + projection_ms + tx_duration_ms

active_process_ms   = total_processing_ms

node_efficiency_pct = 100 * active_process_ms / total_residency_ms

total_residency_ms  = last_compressed_dpdk_egress - first_point_arrival
```

When `OFFLOAD_MODE_ENABLED` acquires a comprehensive final `geo_agg_hdr` whose `active_point_count` aligns accurately with the assembled point population:

```text
geometry_aggregation_ms = 0
max_r_ms                = 0
```

This occurs because the `Encoder`  reliably consumes the centroid, extents, bounding-box centre, &  ultimate radius computed upstream. Provided offload remains disabled, or  should metadata prove unacceptable, identical geometry undergoes local  recomputation. This explicit toggle supplies the rigorous control  requisite to critically evaluate the computational-placement advantage,  rather than summarily presuming its efficacy.

The workload controller relies on the subsequent formulations:

```text
T_base           = 1000 / TARGET_FPS
T_budget( skip ) = skip * T_base
E_n              = EWMA_ALPHA * service_n + ( 1 - EWMA_ALPHA ) * E_( n - 1 )
workload_ratio   = E_n / T_budget( skip )
```

utilising the following constants:

```text
EWMA_ALPHA        = 0.25
OVERLOAD_RATIO    = 0.90
RECOVERY_RATIO    = 0.75
OVERLOAD_FRACTION = 0.25
RECOVERY_FRACTION = 0.10
OVERLOAD_STREAK   = 2
RECOVERY_STREAK   = 9
MAX_SKIP          = 9
```

The functional overload predicate equates to:

```text
overloaded = workload_ratio >= 0.90 OR wait_raw_queue_ms > 0.25 * active_budget_ms OR frame_backlog >= 2
```

`codec_backlog` is logged as a critical diagnostic but decidedly does **not**  interface directly with this boolean. This segregation gains profound  relevance when interpreting subsequent forced-stress evaluations.

Conclusively, `encode_h265_ms` must preserve absolute independence from `total_processing_ms`:  the former culminates upon detecting the initial video-"PES" output  after the frame penetrates the persistent "FFmpeg" writer, whereas `total_processing_ms`  inherently amalgamates the "I420" writer interval defined by the  foundational node-processing boundary. Merging these values would  erroneously conflate disparate asynchronous stages.

### 9.16 Encoder "End-of-Stream" Handling

Upon detecting "EOS", the `Encoder`  initiates the finalisation of any eligible residual frame, drains  lingering writer commitments & encoded outputs, seals the "FFmpeg"  input channel, completes the parser exhaustion process, flushes residual  "DPDK" output, & solely thereafter records the terminal telemetry  data.

This sequential ordering remains essential  because encoded bytes frequently linger within the "codec" / muxer  pipeline long after the concluding application frame submission.  Consequently, the terminal frame can routinely exhibit elevated  residency or compressed-media delays in contrast to steady-state frames;  thus, it demands analysis as an authentic tail-drain circumstance  rather than suffering unceremonious deletion.

---

## 🖥️ 10. "CPU", "GPU", & Core-Constrained Execution

The launcher implements an eight-logical-"CPU" execution topology ( `0-7`  ). Core allocation remains an integral component of the overarching  experiment & must be rigorously preserved when attempting  comparative evaluations.

| **Logical "CPU"** | **Assignment in Current Launcher**                                                                                   | **Rationale**                                                                                              |
| ----------------- | -------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `0`               | "OVS" auxiliary "lcore" + Encoder "FFmpeg" affinity; additionally incorporated within the forthcoming Decoder cpuset | Represents a pragmatic housekeeping / "codec" compromise upon the constrained host architecture            |
| `1`               | "OVS"-"DPDK" "PMD"                                                                                                   | Furnishes dedicated virtual-switch data-path processing                                                    |
| `2`               | Camera                                                                                                               | Constitutes a dedicated Camera "DPDK" worker                                                               |
| `3`               | `SFF1`; the anticipated `SFF3` container adopts an identical cpuset                                                  | Exclusively `SFF1` maintains activity within the validated upstream hot path                               |
| `4`               | `SFF2`                                                                                                               | Serves as a dedicated four-port forwarder / proxy unit                                                     |
| `5`               | Encoder                                                                                                              | Acts as the primary Encoder "DPDK" / "C++" execution core                                                    |
| `6`               | Decoder                                                                                                              | Safeguarded for future Decoder processing operations; the Decoder container concurrently accesses Core `0` |
| `7`               | User                                                                                                                 | Safeguarded for subsequent rendering / client interaction tasks                                            |

The extant "OVS" configuration specifies:

```text
dpdk-"lcore"-mask = 0x1   -> Core 0
pmd-cpu-mask    = 0x2   -> Core 1
dpdk-socket-mem = 1024 MiB
```

The host launcher designates:

```text
2048 * 2 MiB HugePages = 4096 MiB ~= 4 GiB
```

### Core 0 Sharing

Anchoring "FFmpeg" upon Core `0`  constitutes a deliberate architectural concession. This manoeuvre  effectively prevents the "codec" writer / child process from  monopolising the exclusive `Encoder`  "DPDK" core, although it inevitably fosters contention involving "OVS"  auxiliary functions & broader host housekeeping activities.

Existing  telemetry substantiates that this arrangement proves adequate for the  validated upstream execution, yet it undeniably warrants critical  reassessment upon `Decoder` activation, noting the `Decoder` container identically envelops Core `0` & Core `6`.

### Why the Current Constraint Is Manageable

The present implementation mitigates pressure upon the restricted "CPU" cluster via:

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

The  finalised holistic chain will encompass markedly greater simultaneous  activity compared to the current upstream subset. Consequently, "CPU"  placement inherently constitutes an element of the experimental  methodology rather than presenting an irrevocably resolved scaling  dynamic.

---

## 11. "HugePages" & Optional "CPU" Isolation

The experiment prescribes explicit "DPDK" memory configuration antecedent to topology fabrication.

The current `init_all.sh` dictates an allocation of:

```text
nr_hugepages  = 2048
HugePage size = 2 MiB
total         = 4 GiB
```

Simultaneously, it expunges filesystem caches prior to environment activation. `WARM_MODE_ENABLED` is sequentially instituted by the `Camera` itself; therefore, both the host cache-reset policy & the `Camera` warm-mode policy must be diligently documented during any attempt to replicate a given execution.

The supplementary isolation scripts provision:

```text
isolcpus=1-7
```

thereby isolating Core `0` as the singular non-isolated housekeeping "CPU".

Standard execution from `src/` involves:

```bash
sudo ./enable_isolcpus.sh
sudo reboot
```

Following the reboot:

```bash
cat /proc/cmdline
```

must be executed to confirm the robust presence of `isolcpus=1-7`.

Reinstating the default scheduler paradigm necessitates:

```bash
sudo ./disable_isolcpus.sh
sudo reboot
```

The customary `init_all.sh` / `stop_all.sh` lifecycle explicitly omits toggling the reboot-tier `isolcpus`  parameter. "CPU" isolation, "Docker" cpusets, "OVS" "PMD" designation,  & "codec" affinity must unconditionally be reported as wholly  independent experimental constraints.

---

## 12. Container & "OVS"-"DPDK" Environment

The repository documentation strictly retains the universal build environment foundational to the project:

```text
nvidia/cuda:12.2.0-devel-ubuntu22.04
"DPDK" 22.11.4
```

This specific image additionally supplies "GCC" / "G++", "Meson", "Ninja", "NUMA" development dependencies, "FFmpeg", `tcpdump`, `ethtool`,  & the vital utilities prerequisite for the current launch scripts. These versioned parameters should be meticulously archived corresponding  to each benchmark, thereby safeguarding against subsequent base image  alterations.

Native service functions initiate with the following directives:

```text
--net none
--privileged
```

The distinct nodes articulate exclusively through explicitly mounted "DPDK" "vhost-user" sockets, deliberately bypassing the conventional "Docker"  network stack.

Each corresponding service container assumes relevant mounts encompassing:

```text
/dev/hugepages
/tmp
/shared
/app
```

alongside a discrete `DPDK_CORE` specification. The core project source directory mounts directly as `/app`,  guaranteeing that node entrypoints consistently execute the extant  repository source rather than languishing upon a stagnant source tree  entrenched within a historical container image.

The launcher institutes implemented nodes sequentially, diligently awaiting  the manifestation of their indispensable "vhost-user" sockets preceding  progression. This methodical pace mitigates topology-startup collisions  & decisively exposes active data-path bindings.

### Build-Time Specialisation

The `Encoder` build definitively preserves a performance-oriented posture. The specific compilation profile ratified by the repository snapshot  entails:

```text
"C++"  : -O3 -march=native -ffast-math -funroll-loops -std=c++14
"CUDA" : -O3 -arch=sm_61 -std=c++14
```

These explicit flags fundamentally construct the experimental condition. `-march=native` rigorously tailors host object code precisely to the build infrastructure, `-ffast-math` sanctions non-conservative floating-point permutations, & `-arch=sm_61`  rigidly locks the resultant device target. Consequently, any benchmark  reconstructed using a disparate processor, compiler, or "GPU"  architecture irrevocably relinquishes any presumptive claims to  performance or bit-level parity.

Any benchmark posturing for legitimate comparison must unequivocally archive:

```text
compiler versions
host optimisation flags
"CUDA" architecture target
"GPU" model / driver
"FFmpeg" / "NVENC" version
"DPDK" / "OVS" versions
```

### "OVS"-"DPDK" Topology

The virtual switch engenders a singular `netdev` bridge:

```text
br-sfc
```

& implements explicit adjacent "vhost-user" bindings:

```text
Camera <-> SFF1 <-> SFF2 <-> Encoder
                     |  |
                     |  +-> Decoder
                     +----> SFF3 <-> User
```

"OpenFlow"  regulations meticulously connect singularly those declared adjacent  endpoints. A definitive default-deny mandate conclusively drops any  unmatched traffic.

This rigorous separation is critically imperative: "OVS"-"DPDK" guarantees deterministic connectivity, whereas `SFF1` & `SFF2` engineer application-cognisant service computation, profound service-path classification, & exact proxy semantics.

The topology script maintains an optional provision to activate internal mirror ports aimed at `tcpdump`  inspection. Debug mirroring decisively alters the operational  environment & should consistently remain dormant during benchmark  runs unless the specific intention is to study packet capture overhead  in isolation.

---

## 13. Repository Structure

The current repository organisation delineates as follows:

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

The designated experimental point-cloud resource relies upon the **"Loot" sequence traversing the 8i Voxelized Full Bodies ( 8iVFB v2 ) dataset**, graciously provided by 8i Labs & exhaustively catalogued through the JPEG Pleno database.

The foundational dataset establishes four diverse dynamic full-body sequences:

```text
longdress
loot
redandblack
soldier
```

Each  sequence presents a comprehensive human subject meticulously captured  via 42 "RGB" cameras systematically configured within 14 clusters,  capturing at 30 frames/s for an approximate duration of 10 s. The  depth-10 structure necessitates a `1024 x 1024 x 1024` voxel grid, wherein "RGB" colour attributes are rigorously assigned to occupied voxels.

The focal experiment employs the absolute 300-frame depth-10 "Loot" sequence:

```text
loot_vox10_1000.ply
...
loot_vox10_1299.ply
```

The rigorously documented prepared "Loot" dataset snapshot confirms the following metrics:

| **Dataset quantity**                   | **Current "Loot" snapshot**      |
| -------------------------------------- | -------------------------------- |
| Frames                                 | `300`                            |
| Total points                           | `238,146,391`                    |
| Mean points / frame                    | `793,821`                        |
| Minimum points / frame                 | `770,822`                        |
| Maximum points / frame                 | `835,458`                        |
| Aggregate source PLY footprint         | `5,144,378,340 B` ( `5.144 GB` ) |
| Aggregate generated BIN footprint      | `3,810,342,256 B` ( `3.810 GB` ) |
| BIN footprint reduction vs. source PLY | `25.93 %`                        |
| Mean offline conversion time           | `6.310 s / frame`                |

The  entire dataset remains publicly accessible via the JPEG Pleno database  compliant with the attached 8i license protocols. The formally required  academic citation demands:

> E. d'Eon, B. Harrison, T. Myers, & P. A. Chou, *8i Voxelized Full Bodies — A Voxelized Point Cloud Dataset*, ISO/IEC JTC1/SC29 Joint WG11/WG1 input document WG11M40059/WG1M74006, Geneva, January 2017.

Repository  stakeholders must imperatively consult the primary dataset portal &  corresponding license prior to any utilisation or subsequent  redistribution:

```text
[https://plenodb.jpeg.org/pc/8ilabs/](https://plenodb.jpeg.org/pc/8ilabs/)
```

### 14.2 Repository Data Policy

Conspicuously, neither the authentic `.ply` frames nor the procedurally generated `.bin` frames hold presence within this repository's commit history.

This outcome is strictly intentional: the exhaustive "Loot" PLY series scales to approximately `5.14 GB` locally, while the compact binary derivative persists at roughly `3.81 GB`.  Excluding both manifestations from Git rigorously guarantees a  streamlined repository architecture & actively prevents fundamental  source-control mechanisms from succumbing to immense experimental data  weights.

Consequently, the repository securely houses the **code, data schematics, procedural conversion paradigms, & overarching telemetry**, operating on the presumption that voluminous dataset artefacts will be either independently procured or locally generated.

This  capacity-driven repository policy strictly operates independent of the  official dataset licence. Any local replication or expansive  redistribution concerning 8i assets remains unequivocally bound by the  explicit licence appended to the original dataset.

### 14.3 Binary Representation Used by the Camera

The offline converter methodically transfigures every PLY frame into a seamless, header-less array congruent with `point_tx`:

```text
float32 x
float32 y
float32 z
uint8   r
uint8   g
uint8   b
uint8   padding
```

Consequently:

```text
bytes_per_point = 16
```

This robust transformation effectively isolates PLY interpretive parsing & distinct per-field numeric conversions from the `Camera`'s high-frequency streaming conduit. It definitively serves as a **storage / parsing preparatory sequence**,  unequivocally void of aspirations mimicking compression algorithms  grounded in rigorous information theory or explicit rate-distortion  frameworks.

The prevalent scale factor anchored within the repository mandates:

```text
SCALE_FACTOR = 1.0
```

### 14.4 "Python" Environment

The definitive root-level directory:

```text
env/
```

contains  the precise "Python" virtual configuration requisite for the current  offline utilities, most notably the point-cloud translation apparatus.

Activation from the repository root unfolds as:

```bash
source env/bin/activate
```

The existing README snapshot asserts the requisite converter dependencies encompass:

```text
numpy
plyfile
```

Should an environment reconstruction become necessary:

```bash
python3 -m venv env
source env/bin/activate
python -m pip install --upgrade pip
python -m pip install numpy plyfile
```

Libraries encompassing `pandas` & `matplotlib`  definitively lack prerequisite status regarding the execution of the  primary documented converter. Although exceptionally competent regarding  elevated telemetry analysis, they decisively remain extraneous to the  native "DPDK" operational pathway.

### 14.5 Offline Converter

The standard execution paradigm manifests as:

```bash
source env/bin/activate
python3 src/shared/py/converter/converter.py
deactivate
```

The converter operates exclusively as an **offline preparation stage**.  The resultant elapsed chronology, encompassing both PLY ingestion &  BIN extrusion, must definitively eschew amalgamation with `Camera`, SFF, `Encoder`, "CUDA", or explicit "codec" latency quantifications.

Nonetheless,  the converter telemetry presents substantial utility for replicability  parameters, flawlessly tracking the strict frame population &  precisely contrasting the source against the generated data footprint  prevalent throughout the experiment. The definitive source-configured  schema embodies:

| **Metric**         | **Unit / Type** | **Exact Meaning**                                                                                                                                                                      |
| ------------------ | --------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `filename`         | string          | Source `.ply` file subjected to offline processing operations.                                                                                                                         |
| `status`           | string          | `success` dictates the successful culmination of the conversion path lacking exception; alternatively `error`.                                                                         |
| `num_points`       | points          | Aggregate vertices extracted from the PLY `vertex` entity & transcribed to the designated fixed-width binary format.                                                                   |
| `read_ascii_ms`    | ms              | Absolute wall-clock interval consumed exclusively within `PlyData.read( file_path )`.                                                                                                  |
| `write_bin_ms`     | ms              | Absolute wall-clock interval committed to committing the already established contiguous `network_array` toward the `.bin` output.                                                      |
| `conversion_ms`    | ms              | Comprehensive  file interval bridging initial processing steps, overarching parsing,  coordinate / colour abstraction, structural record formulation, &  subsequent output management. |
| `size_ascii_bytes` | bytes           | Exact dimensions of the foundational `.ply` artefact, sourced through `os.path.getsize()`.                                                                                             |
| `size_bin_bytes`   | bytes           | Exact dimensions of the synthetic fixed-width `.bin` artefact, similarly sourced through `os.path.getsize()`.                                                                          |

`conversion_ms` is purposefully broader in scope compared to the strict calculation of `read_ascii_ms + write_bin_ms`. The deviation successfully accommodates point-array segregation, active `SCALE_FACTOR`  implementation, intrinsic numeric casting, meticulous padding  interpolation, & the comprehensive mapping of the 16-byte  architectured representation. Consequently, it must **never** be interpreted merely as the rudimentary summation of the two distinct I / O probes.

The  robust streaming outcomes explored within Section 18 inaugurate  exclusively from the synthesised binary array. Ergo, Offline Converter  chronologies are systematically expunged from all `Camera` / `SFF1` / `SFF2` / `Encoder`  latency metrics & hold relevance strictly parallel to the  correlative Converter telemetry tied to the dataset-preparation epoch.

---

## 🚀 15. Running the Experiment

### 15.1 Obtain & Prepare the Dataset

1. Successfully  acquire the 8iVFB v2 dataset via the official / JPEG Pleno portal &  meticulously retain all associative licence directives.
2. Manually situate the localised "Loot" PLY frames within the repository's dedicated data directory.
3. Initiate the offline converter via the root directory:

```bash
source env/bin/activate
python3 src/shared/py/converter/converter.py
deactivate
```

The authentic `.ply` & structurally generated `.bin` datasets actively bypass versioning paradigms within Git, directly respecting their multi-gigabyte scale.

### 15.2 Optional: Enable "CPU" Isolation

Operating from `src/`:

```bash
sudo ./enable_isolcpus.sh
sudo reboot
```

Following the reboot sequence, re-enter the repository & definitively verify `/proc/cmdline` preceding any benchmarking endeavours.

### 15.3 Start the Environment

Operating from `src/`:

```bash
sudo ./init_all.sh
```

The  designated launcher assumes supreme responsibility concerning the  host-side "DPDK" preliminary preparations & executing the  interconnected topology / container start protocols. Provided `init_all.sh`  sustains active deployment, infrastructural scripts mandate abstention  from redundant initiations unless the user strategically dictates  comprehensive teardown & subsequent reconstruction of the topology.

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

"CPU"  isolation acts intrinsically as a reboot-tier variable, preserving its  absolute independence relative to the conventional container / "OVS"  shutdown sequences.

---

## 16. Entrypoint Execution Model

Every manifested native node is explicitly driven by a dedicated `entrypoint.sh` stationed within its specific container.

The prevailing entrypoint sequence typically:

1. compiles the present mounted source enacting stringent optimisation parameters;
2. formulates precise links against the "DPDK" frameworks embedded within the communal image;
3. deploys the "EAL" upon the specific logical core delineated via `DPDK_CORE`;
4. fabricates one or more `virtio_user` functional devices;
5. rigidly binds them to the correlating `/tmp/vh-*` "vhost-user" sockets;
6. triggers the node's central run-to-completion processing loop.

For illustrative purposes, the `Camera` compiles via a significantly optimised "GCC" command, interconnected using `pkg-config`, & immediately launches operating strictly against its "virtio-user" "vhost" endpoint.

This  dynamic runtime compilation methodology possesses immense utility  across the thesis developmental cycle, guaranteeing the mounted source  firmly retains its status as the exclusive, authoritative structural  manifestation.

---

## 📊 17. Native Telemetry Files

The immediate snapshot authoritatively disseminates native telemetry targeting:

```text
src/shared/log/
```

The preeminent documented files encompass:

| **Component** | **Telemetry**                                                                        |
| ------------- | ------------------------------------------------------------------------------------ |
| Converter     | `log/converter/telemetry_converter.csv`                                              |
| Camera        | `log/camera/telemetry_camera.csv`                                                    |
| SFF1 / "GAC"  | `log/sff1/telemetry_sff1.csv`                                                        |
| SFF2 Route 0  | `log/sff2/telemetry_sff1_enc.csv`                                                    |
| SFF2 Route 1  | `log/sff2/telemetry_enc_dec.csv`                                                     |
| SFF2 Route 2  | `log/sff2/telemetry_dec_sff3.csv` ( path reserved; quantitative semantics deferred ) |
| Encoder       | `log/encoder/telemetry_encoder.csv`                                                  |
| "FFmpeg"      | `log/encoder/ffmpeg.txt`                                                             |

The deeply archived validation arrays comprise 300 distinctive frame rows directed to `Camera`, `SFF1`, `SFF2` Route 0, `Encoder`, & `SFF2` Route 1. Their prevailing schematics present strictly **23**, **36**, **36**, **54**, & **36**  columns respectively. These comprehensive files command authoritative  quantitative foundational status pertinent to Section 18, whereas the  isolated `ffmpeg.txt` stream provides an independent "codec"-centric `vstats` verification dynamic.

### 17.1 Exact CSV Schema Reference

The  subsequent lists meticulously replicate the precise headers inherent  within the validated CSV logs. They feature purposeful verbosity: the  README functions fundamentally as a rigorous measurement primer,  dictating that field intent must remain decisively unambiguous without  necessitating deeper source code interpolation. Elaborate computational  architectures & exact boundaries are presented across Sections 6.6,  7.6, 8.7, & 9.15.

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

`SFF2`  Route 2 robustly sustains a designated projected conduit, yet its  overarching application-body pact & precise route-centric conclusive  parameters decidedly resist stabilisation. Consequently, the literal  creation of `telemetry_dec_sff3.csv` definitively does **not** signify the authentication of a fully fledged 36-field Route-2 quantitative data array.

### 17.2 "FFmpeg" `vstats` Field Semantics

The unyielding "FFmpeg" background instance generates an autonomous `vstats`  registry documenting each coded ingress frame, resolutely spanning the  private pre-roll elements. The variables articulated within `ffmpeg.txt` demand rigorous interpretation via the ensuing rubric:

| **Field** | **Unit / Type**                      | **Meaning**                                                                                                                                                                                                                                                 |
| --------- | ------------------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `out`     | index                                | The  "FFmpeg" output-stream classification corresponding to the statistic.  Given the current framework processes singularly a solitary "MPEG-TS"  output, the factor fundamentally rests at `0`.                                                           |
| `st`      | index                                | Defines the output stream catalogue marker. The active visual-centric channel consistently returns stream `0`.                                                                                                                                              |
| `frame`   | count                                | Chronological  enumerated sequence parameter native to the resolute "FFmpeg" context,  therefore actively absorbing covert pre-roll integrations.                                                                                                           |
| `q`       | encoder-reported quantiser indicator | Quantisation  integer explicitly conveyed via "FFmpeg" identifying the distinct  rendered frame. Operative underneath "NVENC" CBR orchestration, it  functions chiefly as a diagnostic instrument rather than a structurally  rigid constant-QP imposition. |
| `f_size`  | bytes                                | Synthesised frame payload as documented precisely by `vstats`, antecedent to the repository’s overarching frame-ascribed "MPEG-TS" byte ledger dynamics.                                                                                                    |
| `s_size`  | bytes / displayed size               | Incremental cumulative payload recorded via "FFmpeg" up through the contemporaneous boundary.                                                                                                                                                               |
| `time`    | seconds                              | Chronological media timeline synchronised fundamentally alongside the prevailing frame component.                                                                                                                                                           |
| `br`      | kbit/s                               | Strictly frame-focused bitrate marker formulated explicitly by the "FFmpeg" architecture.                                                                                                                                                                   |
| `avg_br`  | kbit/s                               | Compounded mean bitrate diagnostic registered through "FFmpeg" spanning directly up to the operative metric boundary.                                                                                                                                       |
| `type`    | frame type                           | Coded image specification ( e.g., `I`, `P`, or `B` ). The dominant architectural setup universally nullifies B-frame generation.                                                                                                                            |

The  15 pre-roll documentation entries universally warrant retention whilst  interrogating "codec" acceleration dynamics, albeit demanding stringent  expulsion when quantifying fundamental application-frame boundaries.  Contrariwise, `mpeg_bytes_generated` strictly identifies the `Encoder`-ascribed  "MPEG-TS" data density expressly assigned back towards a genuine source  entity, hence generating a metric inherently surpassing `f_size` owing directly to the integration of obligatory transport-stream / muxing protocols.

### 17.3 Timing Quantities Must Not Be Added Indiscriminately

The  structural telemetry purposefully bifurcates parameters encompassing  pure wall-clock tenure, active execution cycles, asynchronous "codec"  emission delays, "GPU" hardware chronological variables, & strict  local Tx-ring engagement metrics.

Standardised node tenure is derived fundamentally as:

```text
residency = node_exit - node_entry
```

However,  the categorical entry / exit definitions depend inextricably upon the  designated functional entity. Distinctly, contemporary `Encoder`  tenure bridges the initial definitive point assimilation toward the  ultimate encoded "DPDK" egress actively registered back to that very  frame.

Prevailing active-process paradigms definitively formulate as:

```text
Camera active_process_ms  = disk_io_ms + serialization_ms + tx_duration_ms

SFF1 active_process_ms    = accumulated packet / geometry / Tx work
SFF2 active_process_ms    = accumulated route processing / Tx work

Encoder active_process_ms = conversion_ms + geometry_aggregation_ms + max_r_ms + projection_ms + tx_duration_ms
```

Consistently targeting all functional components:

```text
node_efficiency_pct = 100 * active_process_ms / total_residency_ms
```

while strictly tethered directly to the function-centric tenure constraints illuminated previously.

Throughput fundamentally entails an exacting semantic realm. `Camera` throughput exclusively leverages its distinct framing transmission corridor. `SFF1` / `SFF2` / `Encoder`  ingress throughput leverages the overarching first-to-terminal  ingestion window governing the correlative logical payload scope. `logical_bitrate_mbps` consciously shuns reiterative structural envelopes, whereas `network_bitrate_mbps` precisely embraces the repeated network superstructure intrinsically tethered to the functional node in question.

Several parameters definitively warrant rigorous separation:

```text
active_tx_ms        -> execution inside local rte_eth_tx_burst() invocations
tx_duration_ms      -> extensive wall-clock transmission / distinct writer duration
encode_h265_ms      -> "FFmpeg" source ingress spanning through primary identified video "PES" detection
"GPU" event metrics -> comprehensively asynchronous device-stage demarcations
```

With specific regard to the `Encoder`:

```text
total_processing_ms = active_process_ms
```

dictates the present framework, yet:

```text
encode_h265_ms
```

remains profoundly asynchronous & strictly must **not** be integrated sequentially within the processing composite calculation.

Equally, `wait_render_queue_ms`  solely calibrates the post-projection stagnation preceding writer  commencement, while slot appropriation metrics & foundational  raw-frame queuing paradigms are purposefully illustrated within discrete  channels.

Ultimately, `Encoder` `end_to_end_latency_ms` currently terminates abruptly at the `Encoder` compressed-media egress boundary. This definitively establishes a partial `Camera`-to-`Encoder`-output marker & unconditionally must not be arbitrarily mischaracterised as the definitive global `Camera`-to-`User` E2E chronological metric.

---

## 🧪 18. Relevant Outcomes from the Validated Snapshot

The quintessential representative telemetry explicitly houses **300 frame records** traversing `Camera`, `SFF1`, `SFF2` Route 0, `Encoder`, & `SFF2`  Route 1. The seamlessly corresponding "FFmpeg" ledger captures the 15 secluded pre-roll units coupled with the identical 300 primary  application targets.

The fully validated upstream structure maps precisely as:

```text
prepared 8i "Loot" BIN
-> Camera
-> SFF1 ( "GAC" )
-> SFF2 ( Route 0 / proxy )
-> Encoder
-> SFF2 ( Route 1 )
```

The underlying measurements definitively elucidate the precise repository  snapshot actively governed by the structural configuration housed inside  this release:

```text
K_FRAMES               = 300
TARGET_FPS             = 30
Camera CACHE_MODE      = CACHE_MODE_MIDDLE
Camera WARM_MODE       = WARM_MODE_ENABLED
POINTS_PER_PACKET      = 80
Encoder OFFLOAD_MODE   = OFFLOAD_MODE_ENABLED
TEMPORAL_ADAPTATION    = TEMPORAL_ADAPTATION_ENABLED
H2D_CHUNK_POINTS       = 65536
YUV_BUFFER_COUNT       = 3
"NVENC" target bitrate = 10M
"NVENC" buffer size    = 20M
"GOP"                  = 15
pre-roll               = 15 frames
```

Absolutely  all 300 procedural frame markers successfully complete processing  across every ratified native unit. No detectable frame-integrity  attenuation presents across the established overarching upstream  trajectory.

### 18.1 Dataset & Streaming Population

The prevailing `Camera` / SFF / `Encoder` telemetry synchronously attests to:

```text
total source points      = 238,146,391
total point payload      = 3,810,342,256 B
mean points / frame      = 793,821.3
mean point payload/frame = 12.701 MB
Camera point packets     = 2,976,979
mean packets / frame     = 9,923.3
```

Consequently,  the foundational source-point ledger & the core binary density  fundamentally correspond entirely alongside the prepared 16-byte "Loot"  representation precisely detailed throughout the primary dataset  segment.

The previously referenced source PLY / BIN proportional correlation definitively constitutes an **offline representation property**,  strictly disconnected from any actionable streaming compression  validation parameters. The contemporary live verification paradigm  initiates exclusively via the pre-processed binary continuity &  meticulously abstains from integrating conversion durations within any  operational real-time latency formulation.

### 18.2 Camera — Nominal 30-fps Source Operation & Local Backpressure

The extant warm-mode `Camera` infrastructure flawlessly sustains the designated source frequency.

Derived via sequential `timestamp_start_tx` intervals:

```text
mean start-to-start interval = 33.330 ms
median interval              = 33.324 ms
95th percentile              = 34.143 ms
observed source rate         = 30.003 frames/s
```

Primary `Camera` chronologies manifest as:

| **Metric**            | **Mean**    | **Median**  | **95th percentile** |
| --------------------- | ----------- | ----------- | ------------------- |
| `disk_io_ms`          | `1.966 ms`  | `1.907 ms`  | `2.467 ms`          |
| `serialization_ms`    | `1.714 ms`  | `1.783 ms`  | `1.929 ms`          |
| `tx_duration_ms`      | `11.930 ms` | `11.918 ms` | `12.967 ms`         |
| `active_tx_ms`        | `2.169 ms`  | `2.175 ms`  | `2.372 ms`          |
| `active_process_ms`   | `15.609 ms` | `15.543 ms` | `16.464 ms`         |
| `total_residency_ms`  | `15.610 ms` | `15.544 ms` | `16.465 ms`         |
| `node_efficiency_pct` | `99.992 %`  | `99.992 %`  | `99.995 %`          |

The foundational logical & definitive network-rate metrics consolidate toward:

```text
internal_throughput_mbs ~= 1067.247 MB/s
logical_bitrate_mbps    ~= 3048.283 Mbit/s
network_bitrate_mbps    ~= 3243.564 Mbit/s
```

`CACHE_MODE_MIDDLE` decisively imposes a rigorous, timed `fread()` encompassing every distinct frame target, yet the overarching `WARM_MODE_ENABLED`  inherently locks the vital source assets effectively beforehand. The  subsequent quantification must therefore be critically reviewed solely  as a **warmed, directly buffered source configuration**, distinctly rejecting implications suggesting an unaided cold-storage benchmark.

The `Camera` registers pronounced local Tx-ring strain metrics:

```text
mean tx_zero_accepts        = 12,379.96 / frame
mean tx_resubmitted_packets = 394,831.16 / frame
```

yet unequivocally dictates:

```text
tx_partial_accepts = 0
mbuf_starvation    = 0
status             = 1 for all 300 frames
```

ultimately  guaranteeing every formulated data point conclusively navigates toward  the successive operational tier. This definitively substantiates the  premise that the strictly managed local retry heuristic effectively  defuses the fundamental producer / consumer disequilibrium bereft of  inciting overarching upstream application attrition. It definitively  asserts a fundamentally "DPDK" queue-centric anomaly & absolutely  must not be inaccurately described via the terminology of conventional  "UDP" retransmission metrics.

### 18.3 SFF1 / "GAC" — Cost of Moving Geometry into the Data Path

The ratified verification configuration dictates `current_skip = 1`, thereby validating the integration of every distinct source unit.

`SFF1` explicitly outputs:

| **Metric**                  | **Mean**    | **Median**  | **95th percentile** |
| --------------------------- | ----------- | ----------- | ------------------- |
| `geometry_aggregation_ms`   | `2.084 ms`  | `2.077 ms`  | `2.147 ms`          |
| `max_r_ms`                  | `1.067 ms`  | `1.063 ms`  | `1.101 ms`          |
| `active_process_ms`         | `4.066 ms`  | `4.057 ms`  | `4.289 ms`          |
| `tx_duration_ms`            | `19.908 ms` | `19.933 ms` | `21.140 ms`         |
| `total_residency_ms`        | `19.922 ms` | `19.947 ms` | `21.153 ms`         |
| `node_efficiency_pct`       | `20.420 %`  | `20.415 %`  | `21.128 %`          |
| `camera_to_node_latency_ms` | `0.345 ms`  | `0.366 ms`  | `0.380 ms`          |
| `network_jitter_ms`         | `0.222 ms`  | `0.080 ms`  | `1.032 ms`          |

The aggregated primary spatial derivation burden intrinsically approaches:

```text
geometry_aggregation_ms + max_r_ms ~= 3.151 ms / frame
```

subsequently  assumed conclusively by the "GAC" subsystem, formulating essential  calculations concurrently as the foundational data stream accurately  navigates the established network vector.

Complete integrity manifests universally throughout the validated procedure:

```text
Camera Tx points
= SFF1 Rx points
= SFF1 Tx points
= 238,146,391 points
```

simultaneously confirming:

```text
data_integrity_pct = 100 % for all frames
eth_errors         = 0
ipv4_errors        = 0
udp_errors         = 0
nsh_errors         = 0
Tx resubmissions   = 0
```

Mean data ingress capacity nominally measures around `674.766 MB/s`. The decidedly more robust `SFF1`  processing burden measured against historically documented iterations  is stringently deliberate: the essential exacting radius evaluations  have successfully migrated directly from the `Encoder` framework straight into the operative "GAC", precluding their outright elimination entirely from the systemic architecture.

### 18.4 SFF2 Route 0 — Stateful Proxy Cost

Directly addressing `SFF1 -> Encoder`, `SFF2`  currently manages definitive service-header validation, fundamental  proxy-state assimilation, meticulous decapsulation, unembellished-"UDP"  reconstruction operations, dedicated route logging, & subsequent  packet progression.

Quantifiable results demonstrate:

| **Metric**                  | **Mean**    | **Median**  | **95th percentile** |
| --------------------------- | ----------- | ----------- | ------------------- |
| `active_process_ms`         | `0.956 ms`  | `0.908 ms`  | `1.440 ms`          |
| `tx_duration_ms`            | `19.841 ms` | `19.875 ms` | `21.064 ms`         |
| `total_residency_ms`        | `19.847 ms` | `19.881 ms` | `21.071 ms`         |
| `node_efficiency_pct`       | `4.806 %`   | `4.506 %`   | `7.042 %`           |
| `camera_to_node_latency_ms` | `0.434 ms`  | `0.454 ms`  | `0.477 ms`          |
| `network_jitter_ms`         | `0.226 ms`  | `0.086 ms`  | `1.021 ms`          |

The route strictly safeguards complete point fidelity:

```text
SFF1 Tx points
= SFF2 Route-0 Rx points
= SFF2 Route-0 Tx points
= Encoder Rx points
```

extending  uniformly throughout all 300 instances, conspicuously devoid of any  registered communication defects or associated Tx resubmission  imperatives.

Conclusively, the designated proxy structure introduces scarcely less than `1 ms` of aggregated measured functional processing delay, concurrently unburdening the subsequent `Encoder` apparatus from processing or stewarding intrinsic service-chain data states.

### 18.5 Encoder — Geometry Offload, "GPU" Work, "Temporal" Controller, & "H.265"

The operational `Encoder` faithfully intercepts every defined data point:

```text
data_integrity_pct  = 100 % for all 300 frames
mbuf_starvation     = 0
Tx resubmissions    = 0
ffmpeg_write_eagain = 0
```

Benefiting from an active `OFFLOAD_MODE_ENABLED` configuration & the concurrent arrival of an irreproachable, complete "GAC" projection matrix:

```text
geometry_aggregation_ms = 0.000 ms for all 300 frames
max_r_ms                = 0.000 ms for all 300 frames
```

This  phenomenon unambiguously yields the most compelling telemetry evidence  validating geometry offload efficacy: the corresponding exacting spatial  formulations surface transparently upstream inside `SFF1` & deliberately vanish entirely out of the `Encoder`'s comprehensive procedural flow.

Principal `Encoder` timing parameters reflect:

| **Metric**                | **Mean**    | **Median**  | **95th percentile** |
| ------------------------- | ----------- | ----------- | ------------------- |
| `conversion_ms`           | `3.350 ms`  | `3.309 ms`  | `3.462 ms`          |
| `geometry_aggregation_ms` | `0.000 ms`  | `0.000 ms`  | `0.000 ms`          |
| `max_r_ms`                | `0.000 ms`  | `0.000 ms`  | `0.000 ms`          |
| `projection_ms`           | `4.122 ms`  | `4.083 ms`  | `4.204 ms`          |
| `tx_duration_ms`          | `8.635 ms`  | `6.377 ms`  | `15.504 ms`         |
| `active_process_ms`       | `16.108 ms` | `13.890 ms` | `29.815 ms`         |
| `total_residency_ms`      | `80.947 ms` | `81.388 ms` | `98.326 ms`         |
| `node_efficiency_pct`     | `20.269 %`  | `17.066 %`  | `32.812 %`          |
| `encode_h265_ms`          | `26.081 ms` | `24.270 ms` | `39.165 ms`         |

"CUDA"-event stratification fundamentally encompasses:

| **Projection component** | **Mean**   |
| ------------------------ | ---------- |
| `gpu_transfer_ms`        | `1.468 ms` |
| `gpu_kernel_ms`          | `0.768 ms` |
| `gpu_packing_ms`         | `0.447 ms` |
| `gpu_copyback_ms`        | `1.390 ms` |
| `host_overhead_ms`       | `0.049 ms` |

Consequently,  the gauged projection window undeniably relinquishes the overwhelming  host-to-device transport dominance pervasive throughout historically  documented framework derivations. Pre-committed buffers, elevated "H2D"  structural chunks, securely pinned exit vectors, static-pose  mathematically explicit confines, & fully assimilated "GPU"  workflows categorically underpin the observed outcome.

The workload administration structure decisively preserves an inactive status amidst the current payload dynamics:

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

This  represents an exceptionally instructive null revelation: the overarching  controller mechanism fundamentally exists, exports reliable metrics,  & yet steadfastly declines to trigger spurious temporal attenuations  when evaluating undeniably proficient processing throughput  capabilities. An exclusive overloading stress regimen intrinsically  remains prerequisite to legitimately confirm subsequent `SKIP+1` / restoration dynamics.

The established isolated `Camera`-to-`Encoder`-egress systemic latency measures at:

```text
end_to_end_latency_ms
mean   = 80.958 ms
median = 81.412 ms
p95    = 98.221 ms
```

Notably,  sequence frame 300 hits an amplified peak tail threshold correlating  precisely alongside the concluding "codec" systematic extraction phase.  Because contemporary residency distinctly ceases strictly contiguous to  the generated "DPDK" egress point, these measurements characteristically  demonstrate an enlarged magnitude contrasting heavily with preceding  abbreviated "FFmpeg"-centric residency indices & thus intrinsically  prohibit a direct 1:1 comparison evaluating purely an "FFmpeg"-input  restriction parameter.

### 18.6 Encoder -> SFF2 Route 1 — Compressed-Media Integrity

The core `Encoder` outputs exactly:

```text
"MPEG-TS" bytes = 10,345,640 B
media packets   = 7,992
```

Simultaneously, Route 1 consistently validates identically:

```text
Rx media bytes = Tx media bytes = 10,345,640 B
Rx packets     = Tx packets     = 7,992
```

pairing resolutely with `data_integrity_pct = 100 %`  representing comprehensively every single data frame completely absent  fundamental networking translation deviations or transmission  malfunctions.

The resulting compressed broadcast-rate variables display stringent congruity binding the `Encoder` explicitly alongside Route 1:

```text
mean logical_bitrate_mbps ~= 8.298 Mbit/s
mean network_bitrate_mbps ~= 9.108 Mbit/s
```

The maiden compressed-media reception traversing `SFF2` triggers exactly at:

```text
camera_to_node_latency_ms
mean   = 51.835 ms
median = 48.903 ms
p95    = 65.138 ms
```

The underlying Route-1 relay system performs microscopically trivial structural "CPU" expenditures:

```text
mean active_process_ms ~= 0.0038 ms
```

Its  fundamental processing latency parameters alongside capacity throughput  spectra inevitably become inherently burst-centric given encoded  "MPEG-TS" arrays consistently trigger asynchronous emissions diverging  sharply from any uniform consistent sequential distribution matrix.  Principally, the concluding data frame intrinsically experiences  undeniable "codec" drain implications. Relating toward this specific  systemic route, data equality validations & primary-media  transmission timelines represent infinitely greater instructional depth  versus simply portraying the mean point-to-point instantaneous ingestion  speed strictly representing a persistent rigid connection bandwidth  ceiling.

### 18.7 "FFmpeg" / "NVENC" Stream Characteristics & Pre-Roll

The explicit definitive `ffmpeg.txt` incorporates uniformly:

```text
315 coded entries total
15 private pre-roll frames
300 application frames
```

Embracing entirely the comprehensive 315-frame aggregate:

```text
I frames = 21
P frames = 294
B frames = 0
```

Eliminating  purely the singular inaugural pre-roll I-frame alongside strictly the  14 associated pre-roll P-frames, the 300 verifiable primary units  maintain precisely:

```text
I frames = 20
P frames = 280
B frames = 0
```

aligning fundamentally harmoniously against the formally established 15-frame overarching "GOP" architecture.

The verifiable application coded-frame data bulk expressly chronicled via "FFmpeg" comprises:

```text
10,051,241 B
```

conversely, the definitive `Encoder` formally ejects:

```text
10,345,640 B "MPEG-TS"
```

The quantitative deviation rests solidly upon:

```text
294,399 B ~= 2.93 % of the coded-frame bytes
```

resolutely  reflecting the pure transport-stream / multiplexer systematic burden  completely disconnected from any genuine overarching application  data-integrity dissonance implications.

The terminal "FFmpeg" cascading systemic average intrinsically mirrors:

```text
avg_br ~= 7.688 Mbit/s
```

encompassing  transparently the fully documented 315-frame sequential breadth  expressly retaining strictly the microscopic unpopulated pre-roll  frames. The structurally delineated `10M`  value functions definitively as a fundamental throughput steering  trajectory parameter, decidedly refraining from constituting any strict  mathematical certainty promising the ultimate sequence universally  hitting precisely the `10 Mbit/s` baseline.

The  proportionate dimensional scale contrasting natively untransformed  basic point payload distributions discharged actively by the initial `Camera` opposed to the eventual extruded "MPEG-TS" capacity intrinsically equates approximately:

```text
3,810,342,256 / 10,345,640 ~= 368.3 : 1
```

This functions strictly as a fundamental **systemic data-capacity proportion parameter** bridging two distinctly separate format methodologies. It is categorically **not**  positioned to impersonate an official point-cloud "codec" algorithm  volumetric ratio benchmark or masquerade as a strictly formalised  rate-distortion experimental determination.

### 18.8 Current Strengths & Limitations

The immediate upstream snapshot simultaneously demonstrates definitively:

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

The snapshot precisely does **not** yet establish:

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

The  contemporaneous architectural revision manifests an undeniably  substantial elevation over the strictly preceding historical repository  structure concerning exclusively criteria wherein the underlying  definitional scope reasonably permits comparative validity.

| **Comparable Quantity**                 | **Preceding Snapshot** | **Current Snapshot** | **Change**              |
| --------------------------------------- | ---------------------- | -------------------- | ----------------------- |
| Observed Camera rate                    | `~8.08 fps`            | `~30.003 fps`        | `3.71 x` ( `+271.3 %` ) |
| Camera `disk_io_ms`                     | `114.279 ms`           | `1.966 ms`           | `-98.3 %`               |
| Camera `total_residency_ms`             | `123.778 ms`           | `15.610 ms`          | `-87.4 %`               |
| Encoder `conversion_ms`                 | `7.748 ms`             | `3.350 ms`           | `-56.8 %`               |
| Encoder `projection_ms`                 | `15.661 ms`            | `4.122 ms`           | `-73.7 %`               |
| Encoder "FFmpeg"-input `tx_duration_ms` | `11.330 ms`            | `8.635 ms`           | `-23.8 %`               |
| Encoder `active_process_ms`             | `34.738 ms`            | `16.108 ms`          | `-53.6 %`               |
| Encoder `encode_h265_ms`                | `259.894 ms`           | `26.081 ms`          | `-90.0 %`               |
| Route-1 `camera_to_node_latency_ms`     | `407.272 ms`           | `51.835 ms`          | `-87.3 %`               |

Two  isolated increases strictly demand accurate interpretation firmly  defined as an inherently purposeful architectural task redistribution  explicitly eschewing mischaracterisation as unintended or inexplicable  performance degradations:

```text
SFF1 active_process_ms : 2.255 -> 4.066 ms
SFF2 R0 active_process : 0.841 -> 0.956 ms
```

`SFF1` definitively commands current responsibility managing primary geometry aggregations encompassing rigorous exact `max_r` parameters seamlessly excised from previous `Encoder` routines. Concurrently, `SFF2`  Route 0 definitively tackles exacting stateful proxy preservation  configurations fundamentally displacing simplistic direct-routing  models. Thus, corresponding processing expansions remain structurally  interconnected with unambiguously novel functionality additions.

The fundamental `Encoder` telemetry outputs unambiguously proffer definitive evidence confirming verifiable spatial structural offload efficiency:

```text
SFF1 geometry + max_r ~= 3.151 ms / frame
Encoder geometry       = 0.000 ms / frame
Encoder max_r          = 0.000 ms / frame
```

Nonetheless, the expansive `Encoder` performance acceleration parameter categorically must **not**  be accredited solely to this isolated functional relocation. The extant  framework concurrently leverages deeply persistent "CUDA" components,  extensive `H2D_CHUNK_POINTS = 65536`  configurations, rigidly static analytical boundaries, a strictly  coalesced projection network, distinctly threefold active "I420"  buffering reserves, a uniquely assigned structural writing vector,  precise "codec" preliminary warming protocols, & inherently  accelerated "FFmpeg" / "NVENC" operational calibrations.

The  present framework implementation therefore showcases definitively  verified substantial engineering acceleration profoundly eclipsing  strictly previous iterations & undeniably arriving at a  significantly elevated operational equilibrium firmly surpassing  fundamentally pure application-stratum topologies initially prompting  rigorous comprehensive revision processes. Any broad macro-scale  operational juxtaposing categorically demands an explicitly **indicative versus strictly controlled systematic benchmark**  framing strictly unless underlying algorithmic configurations, baseline  transport structural guidelines, fundamental framing horizons, &  exacting measurement prerequisites remain thoroughly & definitively  mirrored.

An inherently indispensable methodological parameter dictates the explicit understanding that prevailing `Encoder` aggregate residency distinctly remains **strictly absent**  across the comparison matrix. The terminating baseline metric  exclusively repositioned completely from a strictly legacy  "FFmpeg"-ingress-specific baseline entirely out toward definitive final  compressed "DPDK" datagram emission point, definitively rendering any  simplistic raw percentage comparative formulation inherently distorted.

### 18.10 Complete Statistical View of the Representative Telemetry

The  ensuing exhaustive matrices are directly tabulated employing rigorously  compiled representative 300-row CSV metrics. They maintain their  explicit integral inclusion purely to guarantee the overarching  repository explicitly functions consistently as an unconditionally  robust measurement directory decisively shunning solely curating  distinct hyper-favourable singular data slices.

> **Interpretation rule:**  The systematically aggregated overarching protocol-error configurations  consistently project solely incorporating ultimate maximum thresholds  definitively sidestepping linear basic mathematical row compilation  procedures. Specific local Tx transmission parameters & writer  enumerations persistently retain fundamental frame-centric locality  & logically permit mathematically sound linear combination  structures. Precise fixed timestamp indices securely delineated across  preceding telemetry definitions intrinsically resist mathematical  standardisation derivations solely because an absolute timestamp's  intrinsic dimensional amplitude decisively refuses to portray any  standalone inherent throughput performance narrative.

#### Camera

`status = 1`: **300 / 300**; `current_skip`: **1 -> 300 frames**.

| **Metric**                | **Mean**       | **Median / Final** | **P95 / Max**  | **Maximum / Total** |
| ------------------------- | -------------- | ------------------ | -------------- | ------------------- |
| `tx_points`               | 793,821.303    | 789,877.000        | 817,501.750    | 835,458.000         |
| `tx_packets`              | 9,923.263      | 9,874.000          | 10,219.050     | 10,444.000          |
| `payload_bytes`           | 12,701,140.853 | 12,638,032.000     | 13,080,028.000 | 13,367,328.000      |
| `internal_throughput_mbs` | 1,067.247      | 1,064.210          | 1,144.624      | 1,197.250           |
| `logical_bitrate_mbps`    | 3,048.283      | 3,033.137          | 3,139.217      | 3,208.168           |
| `network_bitrate_mbps`    | 3,243.564      | 3,227.448          | 3,340.318      | 3,413.697           |
| `disk_io_ms`              | 1.966          | 1.906              | 2.467          | 2.575               |
| `serialization_ms`        | 1.714          | 1.783              | 1.929          | 1.983               |
| `tx_duration_ms`          | 11.930         | 11.918             | 12.967         | 13.498              |
| `active_tx_ms`            | 2.169          | 2.175              | 2.372          | 2.452               |
| `active_process_ms`       | 15.609         | 15.543             | 16.464         | 16.973              |
| `total_residency_ms`      | 15.610         | 15.544             | 16.465         | 16.974              |
| `node_efficiency_pct`     | 99.992         | 99.992             | 99.995         | 99.997              |
| `tx_zero_accepts`         | 12,379.960     | 12,218.000         | 14,255.000     | sum = 3,713,988     |
| `tx_partial_accepts`      | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_resubmit_calls`       | 12,379.960     | 12,218.000         | 14,255.000     | sum = 3,713,988     |
| `tx_resubmitted_packets`  | 394,831.157    | 389,248.500        | 454,915.000    | sum = 118,449,347   |
| `mbuf_starvation`         | 0.000          | 0.000              | 0.000          | sum = 0             |

#### SFF1 / "GAC"

`status = 1`: **300 / 300**; `current_skip`: **1 -> 300 frames**.

| **Metric**                  | **Mean**       | **Median / Final** | **P95 / Max**  | **Maximum / Total** |
| --------------------------- | -------------- | ------------------ | -------------- | ------------------- |
| `original_points`           | 793,821.303    | 789,877.000        | 817,501.750    | 835,458.000         |
| `rx_points`                 | 793,821.303    | 789,877.000        | 817,501.750    | 835,458.000         |
| `tx_points`                 | 793,821.303    | 789,877.000        | 817,501.750    | 835,458.000         |
| `rx_packets`                | 9,923.263      | 9,874.000          | 10,219.050     | 10,444.000          |
| `tx_packets`                | 9,923.263      | 9,874.000          | 10,219.050     | 10,444.000          |
| `payload_bytes`             | 12,701,140.853 | 12,638,032.000     | 13,080,028.000 | 13,367,328.000      |
| `data_integrity_pct`        | 100.000        | 100.000            | 100.000        | 100.000             |
| `internal_throughput_mbs`   | 674.766        | 672.360            | 707.873        | 743.697             |
| `logical_bitrate_mbps`      | 3,048.283      | 3,033.137          | 3,139.217      | 3,208.168           |
| `network_bitrate_mbps`      | 3,376.932      | 3,360.155          | 3,477.661      | 3,554.064           |
| `tx_duration_ms`            | 19.908         | 19.933             | 21.140         | 21.552              |
| `active_tx_ms`              | 0.656          | 0.648              | 0.784          | 0.810               |
| `active_process_ms`         | 4.066          | 4.056              | 4.289          | 4.402               |
| `geometry_aggregation_ms`   | 2.084          | 2.077              | 2.147          | 2.196               |
| `max_r_ms`                  | 1.067          | 1.063              | 1.101          | 1.278               |
| `cycle_ms`                  | 33.290         | 33.347             | 33.599         | 34.733              |
| `header_wait_ms`            | 13.368         | 13.373             | 14.664         | 14.984              |
| `total_residency_ms`        | 19.922         | 19.947             | 21.153         | 21.566              |
| `node_efficiency_pct`       | 20.420         | 20.415             | 21.128         | 22.605              |
| `camera_to_node_latency_ms` | 0.345          | 0.366              | 0.380          | 0.475               |
| `schedule_delay_ms`         | -0.346         | -0.236             | 0.027          | 0.090               |
| `network_jitter_ms`         | 0.222          | 0.080              | 1.032          | 1.282               |
| `eth_errors`                | cumulative     | 0                  | 0              | final / max         |
| `ipv4_errors`               | cumulative     | 0                  | 0              | final / max         |
| `udp_errors`                | cumulative     | 0                  | 0              | final / max         |
| `nsh_errors`                | cumulative     | 0                  | 0              | final / max         |
| `tx_zero_accepts`           | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_partial_accepts`        | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_resubmit_calls`         | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_resubmitted_packets`    | 0.000          | 0.000              | 0.000          | sum = 0             |

#### SFF2 Route 0

`status = 1`: **300 / 300**; `current_skip`: **1 -> 300 frames**.

| **Metric**                  | **Mean**       | **Median / Final** | **P95 / Max**  | **Maximum / Total** |
| --------------------------- | -------------- | ------------------ | -------------- | ------------------- |
| `original_points`           | 793,821.303    | 789,877.000        | 817,501.750    | 835,458.000         |
| `rx_points`                 | 793,821.303    | 789,877.000        | 817,501.750    | 835,458.000         |
| `tx_points`                 | 793,821.303    | 789,877.000        | 817,501.750    | 835,458.000         |
| `rx_media_bytes`            | 0.000          | 0.000              | 0.000          | 0.000               |
| `tx_media_bytes`            | 0.000          | 0.000              | 0.000          | 0.000               |
| `rx_packets`                | 9,923.263      | 9,874.000          | 10,219.050     | 10,444.000          |
| `tx_packets`                | 9,923.263      | 9,874.000          | 10,219.050     | 10,444.000          |
| `payload_bytes`             | 12,701,140.853 | 12,638,032.000     | 13,080,028.000 | 13,367,328.000      |
| `data_integrity_pct`        | 100.000        | 100.000            | 100.000        | 100.000             |
| `internal_throughput_mbs`   | 640.645        | 638.266            | 670.649        | 702.078             |
| `logical_bitrate_mbps`      | 3,048.283      | 3,033.137          | 3,139.217      | 3,208.168           |
| `network_bitrate_mbps`      | 3,348.353      | 3,331.717          | 3,448.231      | 3,523.985           |
| `tx_duration_ms`            | 19.841         | 19.875             | 21.064         | 21.471              |
| `active_tx_ms`              | 0.732          | 0.696              | 1.095          | 1.340               |
| `active_process_ms`         | 0.956          | 0.908              | 1.440          | 1.705               |
| `cycle_ms`                  | 33.290         | 33.347             | 33.599         | 34.744              |
| `header_wait_ms`            | 13.442         | 13.462             | 14.733         | 15.044              |
| `total_residency_ms`        | 19.847         | 19.881             | 21.071         | 21.478              |
| `node_efficiency_pct`       | 4.806          | 4.505              | 7.042          | 8.014               |
| `camera_to_node_latency_ms` | 0.434          | 0.454              | 0.477          | 0.580               |
| `schedule_delay_ms`         | -0.368         | -0.262             | 0.000          | 0.056               |
| `network_jitter_ms`         | 0.226          | 0.085              | 1.021          | 1.297               |
| `eth_errors`                | cumulative     | 0                  | 0              | final / max         |
| `ipv4_errors`               | cumulative     | 0                  | 0              | final / max         |
| `udp_errors`                | cumulative     | 0                  | 0              | final / max         |
| `nsh_errors`                | cumulative     | 0                  | 0              | final / max         |
| `tx_zero_accepts`           | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_partial_accepts`        | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_resubmit_calls`         | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_resubmitted_packets`    | 0.000          | 0.000              | 0.000          | sum = 0             |

#### Encoder

`status = 1`: **300 / 300**; `current_skip`: **1 -> 300 frames**; `event`: **IDLE -> 295, WARMUP -> 5**.

| **Metric**                  | **Mean**       | **Median / Final** | **P95 / Max**  | **Maximum / Total** |
| --------------------------- | -------------- | ------------------ | -------------- | ------------------- |
| `original_points`           | 793,821.303    | 789,877.000        | 817,501.750    | 835,458.000         |
| `rx_points`                 | 793,821.303    | 789,877.000        | 817,501.750    | 835,458.000         |
| `tx_points`                 | 793,821.303    | 789,877.000        | 817,501.750    | 835,458.000         |
| `rx_packets`                | 9,923.263      | 9,874.000          | 10,219.050     | 10,444.000          |
| `tx_packets`                | 26.640         | 22.000             | 103.050        | 111.000             |
| `payload_bytes`             | 12,701,140.853 | 12,638,032.000     | 13,080,028.000 | 13,367,328.000      |
| `data_integrity_pct`        | 100.000        | 100.000            | 100.000        | 100.000             |
| `internal_throughput_mbs`   | 643.671        | 640.953            | 674.371        | 706.136             |
| `logical_bitrate_mbps`      | 8.298          | 6.699              | 32.513         | 35.079              |
| `network_bitrate_mbps`      | 9.108          | 7.364              | 35.706         | 38.521              |
| `conversion_ms`             | 3.350          | 3.308              | 3.462          | 11.756              |
| `geometry_aggregation_ms`   | 0.000          | 0.000              | 0.000          | 0.000               |
| `max_r_ms`                  | 0.000          | 0.000              | 0.000          | 0.000               |
| `projection_ms`             | 4.122          | 4.083              | 4.204          | 11.761              |
| `tx_duration_ms`            | 8.635          | 6.377              | 15.504         | 39.221              |
| `active_process_ms`         | 16.108         | 13.889             | 29.815         | 46.829              |
| `total_processing_ms`       | 16.108         | 13.889             | 29.815         | 46.829              |
| `total_residency_ms`        | 80.947         | 81.388             | 98.326         | 340.138             |
| `node_efficiency_pct`       | 20.269         | 17.066             | 32.812         | 51.899              |
| `gpu_transfer_ms`           | 1.468          | 1.465              | 1.525          | 1.999               |
| `gpu_kernel_ms`             | 0.768          | 0.765              | 0.790          | 0.911               |
| `gpu_packing_ms`            | 0.447          | 0.444              | 0.463          | 0.520               |
| `gpu_copyback_ms`           | 1.390          | 1.390              | 1.393          | 1.550               |
| `host_overhead_ms`          | 0.049          | 0.024              | 0.025          | 7.436               |
| `camera_to_node_latency_ms` | 0.011          | 0.029              | 0.072          | 0.175               |
| `end_to_end_latency_ms`     | 80.958         | 81.412             | 98.221         | 340.074             |
| `schedule_delay_ms`         | 19.381         | 19.334             | 20.359         | 20.856              |
| `network_jitter_ms`         | 0.228          | 0.085              | 1.061          | 1.316               |
| `wait_raw_queue_ms`         | 0.003          | 0.002              | 0.012          | 0.018               |
| `wait_render_queue_ms`      | 0.299          | 0.019              | 0.335          | 17.105              |
| `workload_ewma_ms`          | 4.177          | 4.101              | 4.217          | 11.777              |
| `workload_ratio`            | 0.125          | 0.123              | 0.127          | 0.353               |
| `frame_backlog`             | 0.000          | 0.000              | 0.000          | 0.000               |
| `codec_backlog`             | 1.107          | 1.000              | 2.000          | 3.000               |
| `encode_h265_ms`            | 26.081         | 24.270             | 39.165         | 62.008              |
| `mpeg_bytes_generated`      | 34,485.467     | 27,824.000         | 135,378.800    | 146,076.000         |
| `ffmpeg_write_calls`        | 1.000          | 1.000              | 1.000          | sum = 300           |
| `ffmpeg_write_eagain`       | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_zero_accepts`           | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_partial_accepts`        | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_resubmit_calls`         | 0.000          | 0.000              | 0.000          | sum = 0             |
| `tx_resubmitted_packets`    | 0.000          | 0.000              | 0.000          | sum = 0             |
| `mbuf_starvation`           | 0.000          | 0.000              | 0.000          | sum = 0             |

#### SFF2 Route 1

`status = 1`: **300 / 300**; `current_skip`: **1 -> 300 frames**.

| **Metric**                  | **Mean**    | **Median / Final** | **P95 / Max** | **Maximum / Total** |
| --------------------------- | ----------- | ------------------ | ------------- | ------------------- |
| `original_points`           | 793,821.303 | 789,877.000        | 817,501.750   | 835,458.000         |
| `rx_points`                 | 0.000       | 0.000              | 0.000         | 0.000               |
| `tx_points`                 | 0.000       | 0.000              | 0.000         | 0.000               |
| `rx_media_bytes`            | 34,485.467  | 27,824.000         | 135,378.800   | 146,076.000         |
| `tx_media_bytes`            | 34,485.467  | 27,824.000         | 135,378.800   | 146,076.000         |
| `rx_packets`                | 26.640      | 22.000             | 103.050       | 111.000             |
| `tx_packets`                | 26.640      | 22.000             | 103.050       | 111.000             |
| `payload_bytes`             | 34,485.467  | 27,824.000         | 135,378.800   | 146,076.000         |
| `data_integrity_pct`        | 100.000     | 100.000            | 100.000       | 100.000             |
| `internal_throughput_mbs`   | 1,030.174   | 0.842              | 10,241.371    | 15,456.481          |
| `logical_bitrate_mbps`      | 8.298       | 6.699              | 32.513        | 35.079              |
| `network_bitrate_mbps`      | 9.108       | 7.364              | 35.706        | 38.521              |
| `tx_duration_ms`            | 29.694      | 33.255             | 36.052        | 55.581              |
| `active_tx_ms`              | 0.003       | 0.002              | 0.008         | 0.018               |
| `active_process_ms`         | 0.004       | 0.003              | 0.011         | 0.023               |
| `cycle_ms`                  | 34.156      | 33.321             | 66.690        | 322.060             |
| `header_wait_ms`            | 4.460       | 0.000              | 33.456        | 322.052             |
| `total_residency_ms`        | 29.697      | 33.256             | 36.055        | 55.583              |
| `node_efficiency_pct`       | 4.820       | 0.010              | 46.716        | 65.556              |
| `camera_to_node_latency_ms` | 51.835      | 48.903             | 65.138        | 340.635             |
| `schedule_delay_ms`         | -7.950      | -10.877            | 4.651         | 280.257             |
| `network_jitter_ms`         | 2.489       | 0.384              | 8.725         | 288.723             |
| `eth_errors`                | cumulative  | 0                  | 0             | final / max         |
| `ipv4_errors`               | cumulative  | 0                  | 0             | final / max         |
| `udp_errors`                | cumulative  | 0                  | 0             | final / max         |
| `nsh_errors`                | cumulative  | 0                  | 0             | final / max         |
| `tx_zero_accepts`           | 0.000       | 0.000              | 0.000         | sum = 0             |
| `tx_partial_accepts`        | 0.000       | 0.000              | 0.000         | sum = 0             |
| `tx_resubmit_calls`         | 0.000       | 0.000              | 0.000         | sum = 0             |
| `tx_resubmitted_packets`    | 0.000       | 0.000              | 0.000         | sum = 0             |

#### Cross-File Integrity & Source-Cadence Checks

```text
Camera mean start-to-start interval = 33.330057 ms
Observed Camera source rate         = 30.002949 frames/s
Total source points                 = 238,146,391
Total point payload                 = 3,810,342,256 B
Point integrity Camera -> SFF1 -> SFF2 Route 0 -> Encoder = 100 % for all 300 frames
Compressed media integrity Encoder -> SFF2 Route 1        = 100 % for all 300 frames
```

#### "FFmpeg" / "NVENC" `vstats` Cross-Check

The explicitly referenced `ffmpeg.txt` comprehensively contains **315 output statistics rows**:  the private 15-frame pre-roll directly succeeded by 300 genuine  application frames. Consequently, the log rigorously confirms that the  pre-roll methodology is effectively secluded from native application  telemetry, all whilst genuinely warming the persistent "codec" state.

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

`logical_bitrate_mbps` & `network_bitrate_mbps`  delineate distinctly separate accounting paradigms. The logical bitrate  fundamentally encapsulates application data parallel to a singular  frame-level metadata iteration as explicitly defined by the given node;  conversely, network bitrate comprehensively accounts for the repetitive  overarching output envelope situated on each individual packet.

These  separate frameworks should absolutely not be amalgamated into an  ambiguous generic "bitrate" variable whilst neglecting explicit  clarification concerning the exact byte model in use.

### 19.2 `CACHE_MODE` & `WARM_MODE` Are Part of the Experimental Condition

The strictly enforced current source configuration defines as:

```text
CACHE_MODE_MIDDLE
WARM_MODE_ENABLED
```

The `Camera` persistently executes targeted per-frame `fread()`  commands, yet the designated source files are comprehensively  pre-mapped / securely locked universally for the run's duration. The  consequent observed `~1.97 ms` `disk_io_ms` essentially establishes a definitively warm file-backed observation. Implementing a fundamentally cold-cache or outright `CACHE_MODE_WORST`  experiment fundamentally generates a distinctly unique systemic  baseline condition & consequently must decidedly abstain from  comparison prior to receiving explicit categorical labelling.

### 19.3 The Current Run Establishes the Upstream 30-fps Source Point, Not Final Real-Time "QoE"

The rigorously measured `Camera` source cadence operates steadily near:

```text
30.003 frames/s
```

while the comprehensive data-point pathway concluding toward the `Encoder` resolutely persists fundamentally loss-free across the thoroughly representative procedural trace.

This  unequivocally signals a materially stronger architecture juxtaposed  against preceding iterations, though the conclusive downstream chain  persistently maintains an incomplete status. Formulating an unequivocal  overarching real-time declaration definitively necessitates the  activation & integration encompassing the `Decoder`, `SFF3`, `User`, explicit rendering operations, alongside the finalization concerning latency / quality metrics.

### 19.4 Source Scheduling, Descriptor Depth, & Backpressure Are Joint Variables

The active `Camera`  directly governs an uncompromising absolute frame timetable  emphatically rejecting a disconnected autonomous pacing setting. The  fundamental descriptor array depth, parallel to rigidly bounded  zero-accept systematic resubmissions, directly orchestrates how  fundamentally a uniquely selected frame's explicit packet cluster  interacts dynamically alongside localised "vhost" / "OVS" systematic  queues.

Consequently, the substantial `Camera`  zero-accept volume explicitly serves as a distinct operational  observation explicitly defining localised queue saturation pressures  consistently operating adjacent to the rigidly selected 30-fps  parameter. Arbitrarily altering any existing descriptor limitations,  "OVS" operational placements, established retry parameter boundaries, or  the underlying source-cache parameters unavoidably defines an entirely  novel experimental model.

### 19.5 Core Affinity Is Part of the Experiment

Altering definitively any of the succeeding elements fundamentally mutates the fundamental execution topology:

```text
isolcpus state
"Docker" cpusets
"OVS" "lcore" placement
"OVS" "PMD" placement
"FFmpeg" "CPU" affinity
"CUDA" device / architecture target
```

The  expressly tabulated contemporary results persist inherently inseparable  from their unequivocally defined specific "CPU" / "GPU" structured  placement maps.

### 19.6 Route 2 Has Proxy Scaffolding but Undefined Application Semantics

`SFF2`  undeniably procures adequate systemic reservations acknowledging Route 2  & thoroughly encompasses the required primary-path stateful proxy  phase explicitly needed to subsequently re-establish operational base  service metadata steering ultimately toward `SFF3` precisely following a prospective `Decoder` loop.

However, deliberately what decisively remains **not defined** constitutes the exact eventual `Decoder`  terminal application packet architecture, its rigid frame-completion  parameters, definitive exact payload-byte accounting systems, alongside  the precisely corresponding accurate Route-2 telemetry finalisation  mechanics.

Consequently, no overarching systemic correctness nor formal performance declarations are currently purported encompassing:

```text
Decoder -> SFF2 Route 2 -> SFF3
```

explicitly awaiting the firm rigid standardisation confirming that exact communication contract.

### 19.7 Final E2E / "QoE" Is Not Yet Available

The uniquely specific `Encoder` parameter explicitly titled `end_to_end_latency_ms`  simultaneously concludes precisely aligning alongside the absolute  final encoded "DPDK" egress specifically designated mapping directly  toward the respective frame:

```text
Camera Tx -> Encoder compressed-media egress
```

This  represents an intrinsically broader parameter juxtaposed against  strictly earlier rigidly "FFmpeg"-ingress-bounded markers, though it  fundamentally remains conclusively partial.

An eventual overarching definitive evaluation resolutely awaits:

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

The extant repository strictly activates a decidedly more profoundly explicit `nsh_hdr`  + context systematic paradigm coupled securely to an exacting stateful  proxy configuration encircling structurally unaware systemic functions.  Regardless, it inherently preserves its foundational identity strictly  operating as an inherently closed uniquely experimental conceptual  protocol. Universal exhaustive generic "RFC 8300" overarching  interoperability is formally disclaimed.

### 19.9 The "Temporal" Controller Was Not Stress-Activated in This Run

The  dedicated temporal controller definitely remains completely integrated,  accurately emitting vital corresponding telemetry, alongside the  undeniably fully established functional reverse temporal interaction  network. However, every singular one of the 300 dedicated source array  elements unequivocally functions applying rigidly `skip = 1` wherein categorically zero isolated `SKIP+1` / `SKIP-1` event actuations ever transpire.

Therefore, the extant diagnostic trace strictly evaluates robust fundamentally **non-intrusive steady-state equilibrium operations**,  expressly distinct from forced overload adaptive response profiles.  Executing a categorically distinct isolated rigidly controlled stress  implementation clearly represents an absolute prerequisite accurately  capable of measuring genuine precise operational response latency  intervals, systemic stability patterns, absolute oscillation resiliency  boundaries, alongside exact dedicated recovery hysteresis profiles.

### 19.10 Binary Portability Has Two Separate Domains

The fundamentally offline isolated `.bin` explicit dataset array stringently applies the precise strictly little-endian `float32`  dimensional converter structural model. Conversely, the exact live  actively functioning "DPDK" network vector explicitly serialises purely  precisely implemented specific numeric systemic communication field  parameters strictly incorporating explicit point-coordinate dedicated  floating-point unique bit signatures.

A substantially  heterogeneous cross-platform structural deployment categorically  demands rigorously standardising essentially both the exact  fundamentally isolated offline specific structural artefact blueprint  seamlessly coupled with any surviving remaining application specific  architectural constructs categorically before presumptively positing  valid explicit inherent comprehensive cross-machine definitive binary  interchangeability.

### 19.11 Dataset Artefacts Are External to Git

The  precisely isolated explicitly originating PLY structured sequence  concurrently with the categorically synthesised exclusive BIN resulting  array unconditionally eschew versioning commits inherently because each  structurally reflects an unconditionally massive definitive  multi-gigabyte presence.

Reproducibility definitively  commands rigorously maintaining meticulous documentation isolating  securely the precise originating dataset release parameters,  corresponding specific operational array sequence markers, precise  inclusive sequential boundary ranges, precise overarching restrictive  license parameters, designated strictly scaling coefficient bounds,  precise exact dedicated converter revision tracking markers, alongside  the exactly correlative strict fundamental streaming native source  definitive tracking revision absolutely tightly bound alongside  fundamentally all generated resulting exported telemetry strings.

---

## 🛠️ 20. Main Engineering Challenges & Current Solutions

### Limited Logical Cores

**Challenge:** The comprehensive target chain encompasses more concurrent roles than can be allocated strictly independent "CPU" resources on the existing eight-logical-core host.

**Current approach:** Dedicating a singular "OVS"-"DPDK" "PMD", enforcing explicit native-node affinity, leveraging "GPU" / "NVENC" offload, controlling resource sharing for inactive or auxiliary roles, & executing cooperative network servicing within the `Encoder`.

### "DPDK" Backpressure at the Source

**Challenge:** A single point-cloud frame encompasses approximately ten thousand application packets; consequently, even a 30-fps source can transiently exceed the acceptance capacity of a local Tx ring.

**Current approach:** Absolute source scheduling, expansive source & `Encoder` descriptor queues, bounded zero-accept resubmission, abbreviated pause backoffs, & explicit per-frame backpressure metrics. The extant run successfully preserves absolute point integrity despite substantial `Camera` zero-accept activity.

### Sustaining the Nominal 30-fps Operating Point

**Challenge:** The antecedent repository snapshot was inherently source-I/O constrained, operating near `8.08 fps`.

**Current approach:** `WARM_MODE_ENABLED` coupled with the middle file-read strategy mitigates timed source acquisition sufficiently for the `Camera` to sustain approximately `30.003 fps` within the current run. The ensuing challenge shifts from merely achieving the source cadence to preserving this operational equilibrium once `Decoder`, `SFF3`, `User`, dynamic pose, & forced temporal-controller stress operate concurrently.

### Performing Useful Work In-Network

**Challenge:** Not every geometric parameter is packet-local. Progressive sums & extrema exhibit composability, whereas the exact radius relies upon the finalised centroid.

**Current approach:** `SFF1` executes the progressive pass during active forwarding, caches solely the frame-local `XYZ` requisite for the unavoidable exact radius pass, & exports final geometry to the `Encoder`. This sustains the in-path objective without erroneously presuming a mathematically frame-global quantity is accessible from the initial packet.

### Avoiding Preprocessing That Would Invalidate the Data-Plane Question

**Challenge:** Exact final geometry could be transmitted immediately only if an alternate element computed it prior to the "GAC" observing the stream.

**Current approach:** Rejecting this circumvention for the primary experiment. `Camera` or offline preprocessing remains strictly confined to representation & transport preparation; the geometric service function fundamentally remains the locus where the geometric outcome is derived. Thus, the architecture evaluates the true cost of in-path computation rather than obscuring it upstream.

### Protecting Rx While Computing

**Challenge:** A frame-aware `Encoder` might otherwise abandon its "DPDK" Rx queue whilst conducting local geometry, "H2D" transfers, "CUDA" computations, or "codec" handoffs.

**Current approach:** Implementing packet-arrival conversion, cooperative polling within local fallback loops, `H2D_CHUNK_POINTS = 65536`, a "CUDA" polling callback active during incomplete streams, three "I420" slots, & a dedicated writer thread. The extant telemetry reports a raw-frame backlog of explicitly zero.

### Maintaining Service State Around Unaware Applications

**Challenge:** Compelling every application to parse experimental "NSH" would inextricably link service-chain research to `Encoder` / `Decoder` implementation specifics.

**Current approach:** `SFF2` intercepts primary state, removes service encapsulation preceding unaware functions, advances "SI" within the proxy state, & re-establishes service metadata exclusively when traffic returns to an aware boundary.

### Preserving "MTU" Across Different Traffic Types

**Challenge:** Raw point packets, geometric service metadata, compressed media, temporal commands, & pose commands command divergent envelopes.

**Current approach:** Deriving payload constants strictly from each complete packet architecture: `80 * 16 B` points upstream & `7 * 188 B` TS packets downstream, whilst ensuring control packets remain intentionally compact.

### Correctly Attributing Asynchronous "Codec" Output

**Challenge:** "FFmpeg" pipe reads inherently fail to preserve video-frame boundaries, & "codec" output operates asynchronously relative to input submission.

**Current approach:** Pre-rolling one "GOP", reconstructing fixed TS packets, detecting video "PES" commencements, associating these commencements with the oldest submitted authentic frame, maintaining pre-roll privacy, & draining the "codec" strictly prior to documenting final telemetry.

---

## ✅ 21. Reproducibility Checklist

Every archived benchmark must unequivocally preserve at least the following parameters:

```text
source revision
README revision
host kernel
"CPU" model & logical "CPU" count
"GPU" model
NVIDIA driver
"CUDA" version
"DPDK" version
"Open vSwitch" version
"Docker" version
"FFmpeg" version

HugePage count & size
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

For the current representative run, the defining source & `Encoder` conditions are documented as:

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

A benchmark attains reproducibility exclusively when the empirical result files & the exact configuration generating them are archived concurrently.

---

## 🚧 22. Ongoing Work

Forthcoming efforts should remain categorised into **functional completion**, **performance validation**, & **quality / end-to-end validation**.

### Functional Completion

```text
1. define the Decoder input / output application contract
2. implement "H.265" decoding & geometric reconstruction
3. finalise SFF2 Route-2 payload accounting & telemetry
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
3. quantify temporal-controller response time, hysteresis, & oscillation behaviour
4. compare CACHE_MODE / WARM_MODE combinations as separate source conditions
5. study descriptor depth & Camera zero-accept backpressure sensitivity
6. repeat validated runs & report confidence intervals / dispersion
7. re-evaluate Core 0 sharing once Decoder "codec" work is active
8. measure complete-chain sustained 30-fps behaviour
9. evaluate static-pose "CUDA" assumptions once dynamic pose is introduced
10. isolate the contribution of geometry offload from the other Encoder optimisations
11. evaluate "FFmpeg" / "NVENC" rate-control & latency sensitivity
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

The most immediate architectural dependency resides in the `Decoder` packet contract. Until said representation is irrevocably fixed, Route-2 telemetry should deliberately remain incomplete rather than being artificially populated with speculative byte semantics.

---

## 📚 23. References

1. J. Halpern & C. Pignataro, **"Service Function Chaining" ( "SFC" ) Architecture**, "RFC 7665", IETF, 2015.
2. P. Quinn, U. Elzur, & C. Pignataro, **"Network Service Header" ( "NSH" )**, "RFC 8300", IETF, 2018. The present project adopts its "SPI" / "SI" terminology & architectural concepts but refrains from claiming full "MD-Type-2" wire-format compliance.
3. E. d'Eon, B. Harrison, T. Myers, & P. A. Chou, **8i Voxelized Full Bodies — A Voxelized Point Cloud Dataset**, ISO/IEC JTC1/SC29 Joint WG11/WG1 input document WG11M40059/WG1M74006, Geneva, January 2017.
4. **JPEG Pleno Database**, *8i Voxelized Full Bodies ( 8iVFB v2 ) — A Dynamic Voxelized Point Cloud Dataset*. Dataset repository: `https://plenodb.jpeg.org/pc/8ilabs/`.
5. **"DPDK" Project**, "Data Plane Development Kit" documentation, comprising Ethdev Rx / Tx queue APIs & "virtio-user" configurations.
6. **"Open vSwitch" Project**, "Open vSwitch" & "OVS"-"DPDK" documentation, encompassing "DPDK" "vhost-user"-client ports.
7. **NVIDIA**, "CUDA" Toolkit documentation & NVIDIA Video "Codec" / "NVENC" documentation.
8. **"FFmpeg" Project**, "FFmpeg" exhaustive documentation.
9. Maria Giovanna Lacaria, **Point Cloud Coding for Extended Reality Services**, Master's Thesis, Sapienza University of Rome, Academic Year 2025/2026. This reference serves as the application-level comparison baseline explicitly documented within the project.

---

## Final Note

The preeminent scholarly contribution of this repository resides in the **holistic co-design of packet transport, service steering, & computation**.

The point cloud is not merely relayed between isolated applications. Instead, the `Camera` regulates frame admission according to an `Encoder`-derived workload signal; the "GAC" evaluates geometry whilst packets actively traverse the data path; `SFF2` preserves experimental "NSH" state encompassing unaware applications; & the `Encoder` systematically merges packet-progressive reception with intensive frame-level "CUDA" / "codec" execution.

The extant 300-frame "Loot" snapshot materially supersedes the antecedent repository design: the warm middle-cache source successfully sustains approximately `30.003 fps`, absolute point & compressed-media integrity endure across the validated path, `Encoder`-local geometric operations are fundamentally excised when "GAC" offload parameters validate, projection & initial-"PES" latency witness substantial reduction, & the workload-driven "Temporal" chain is effectively integrated straight to `Camera`-side admission.

Consequently, the resulting platform operates optimally as an empirical study investigating **which volumetric-streaming operations are suitable for in-place execution on a software data plane, which frame-global mathematical dependencies must remain inherently explicit, & how an application-aware processing bottleneck can systematically regulate the source without invoking transport-level retransmissions or relying on user-driven quality interventions**.

The repository deliberately pauses prior to asserting full `Camera`-to-`User` real-time "QoE", a finalised `Decoder`-to-`SFF3` application configuration, generic "RFC 8300" interoperability, or a controlled superiority metric over heterogeneous reference architectures. Such declarations inherently demand the operational downstream implementation & the supplementary validation phases delineated above.