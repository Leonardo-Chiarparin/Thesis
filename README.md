# 🌐 "DPDK"-based "Service Function Chaining" for Real-Time Point-Cloud Streaming
> **Experimental Thesis Project — Sapienza University of Rome**<br>

> A data-plane-oriented architecture for real-time-oriented volumetric point-cloud transport, in-place geometric aggregation, "GPU" projection, & hardware-accelerated "H.265" delivery.

### 👥 Academic Information
**Author:** Leonardo Chiarparin ( Student ID: **2016363** )<br>

**Thesis Supervisor:** Professor Marco Polverini<br>

**Degree Programme:** Engineering in Computer Science<br>

**Institution:** **Sapienza University of Rome**

---

## 📌 Realization Status

This repository serves as an experimental research platform rather than a production-ready "Service Function Chaining" ( "SFC" ) framework. The present snapshot implements & validates the complete `Camera`-to-`User` volumetric chain, the reverse workload-driven "Temporal" control path, the independent `User`-originated "Pose" path, the asynchronous browser bridge, & two deliberately separated validation conditions for runtime behaviour & objective quality.

| Node | Condition | Responsibility |
|---|---|---|
| `Camera` | Validated | "DPDK"-native point-cloud source, warm-mode file acquisition, frame packetisation, absolute scheduling, temporal selection, bounded local Tx resubmission, & source telemetry |
| `SFF1` | Validated | "Geometry-Aware Classifier" ( "GAC" ) implementing packet-progressive spatial aggregation, exact frame-completing radius evaluation, experimental "NSH" metadata insertion, & "Temporal" decapsulation directed to `Camera` |
| `SFF2`<br>( Route 0 ) | Validated | Stateful proxy for `SFF1` -> `Encoder`; validates the aware envelope, preserves proxy state, strips service metadata, & forwards plain geometry-bearing application datagrams |
| `Encoder` | Validated | "SFC"-unaware frame assembly, geometry-offload consumption or local fallback, workload-driven "Temporal" regulation, "CUDA" projection, persistent "FFmpeg" / "NVENC" encoding, "MPEG-TS" attribution, & optional post-stream "luma"-quality evaluation |
| `SFF2`<br>( Route 1 ) | Validated | Proxy-maintained `Encoder` -> `Decoder` transition, compressed-media integrity accounting, & advancement of the primary service state |
| `Decoder` | Validated | "SFC"-unaware persistent hardware-accelerated "H.265" decoding, occupancy erosion, geometric reconstruction, dynamic pose application, output packetisation, & reconstruction telemetry |
| `SFF2`<br>( Route 2 ) | Validated | Stateful `Decoder` -> `SFF3` transition; advances the retained primary state, re-imposes the base service envelope, & records route-specific reconstructed-point telemetry |
| `SFF3` | Validated | Final aware primary-path boundary; validates / removes base service metadata before `User`, while classifying & encapsulating reverse 24-byte "Pose" commands |
| `User` | Validated | Final reconstructed-frame reassembly, shared-memory publication, fire-&-forget pose dispatch, command / browser acknowledgment telemetry, in-memory quality capture, & terminal synchronization |
| "Python" / "WebSocket" bridge | Validated | Asynchronous latest-frame publication with one frame in flight per peer, shared-memory control exchange, browser acknowledgments, & independent "HTTP" serving |
| `Three.js` viewer | Validated | Dynamic point-cloud rendering, keyboard / button interaction, command-to-photon acknowledgment, & on-demand scene refresh |
| `Gauge` | Validated | Post-"EOS" serial geometric assessment using pose reversal, robust "ICP", statistical outlier filtering, symmetric nearest-neighbour metrics, & direct telemetry merging |

The **validated primary route**, designated as **"Main"**, is:

```
Camera -> SFF1 -> SFF2 ( Route 0 ) -> Encoder -> SFF2 ( Route 1 ) -> Decoder -> SFF2 ( Route 2 ) -> SFF3 -> User
```

Control mechanisms remain deliberately separated into two independent logical service paths rather than being consolidated into a monolithic feedback packet:

```
"Temporal" : Encoder -> SFF2 -> SFF1 -> Camera
"Pose"     : User -> SFF3 -> SFF2 -> Decoder
```

The "Temporal" loop is fully operational. `Encoder` derives a requested `temporal_skip`, `SFF2` classifies the plain control datagram & imposes the corresponding service state, `SFF1` validates / removes the envelope, & `Camera` applies the reflected factor before subsequent source-frame admission. In both final representative runs, the controller remains at `current_skip = 1`, because the measured workload does not satisfy the configured overload conditions.

The "Pose" cycle is likewise functional. Browser-originated `yaw`, `pitch`, & `zoom` modifications are written into the `User` control mapping, dispatched as a plain 24-byte payload by the native `User`, encapsulated by `SFF3` as `SPI 300 / SI 255`, stripped by `SFF2`, & consumed by `Decoder`. `Decoder` applies the most recent accepted pose during reconstruction; the corresponding values return with the reconstructed frame & are correlated by `User` with command identifiers & browser render acknowledgments.

> **Repository Note:** The final topology no longer depends upon "Open vSwitch" or an "OVS"-"DPDK" "PMD". Native components are attached directly through explicit `/tmp/sfc-*` "vhost-user" / "virtio-user" adjacencies. This removes an intermediary switching stage & releases the logical core previously dedicated to the virtual switch for application-level placement.

> **Validation Scope:** Two complementary 300-frame "Loot" conditions are archived. `QUALITY_CAPTURE = 0` enables the asynchronous browser bridge & measures interactive end-to-end / command behaviour; `QUALITY_CAPTURE = 1` removes the browser path, captures objective "luma" & reconstructed-geometry information in memory, & evaluates quality strictly after stream termination. Both final runs sustain 300 / 300 complete frames through every primary-route stage at `current_skip = 1`, with a `Camera` source cadence of approximately `29.997 frames/s`.

---

## 🎯 1. Project Motivation & Research Objective

The central thesis objective is to take the volumetric point-cloud pipeline developed in the reference work & redistribute its processing responsibilities over an explicit network-oriented service graph. The contribution is therefore not a new point-cloud codec in isolation, but a concrete "DPDK" / "SFC" realisation of the next step anticipated by the reference architecture: decomposing coarse application blocks into independently measurable, placeable, & steerable functions while preserving the same real-time 300-frame service objective.

This project investigates the feasibility of migrating selected functions within a real-time volumetric streaming pipeline from conventional application-level microservices to direct execution during packet traversal through a "DPDK"-based "SFC".

The objective extends beyond merely substituting kernel sockets with a faster packet-I / O "API". The principal research question addresses whether the forwarding path can function as an active computational component without overloading the Data Plane or compromising the semantic correctness of frame-level processing. This approach aims to preserve bounded queuing behaviour, data integrity, & sufficient observability to accurately attribute latency to the responsible components.

### 1.1 Why "DPDK" & "UDP" Are Design Requirements


"DPDK" is employed because the experiment necessitates more than aggregate throughput. Service functions require the capability to inspect & modify the packet envelope directly, retain per-frame state, perform incremental computations while traffic is in flight, observe local queue acceptance, & account separately for active `rte_eth_tx_burst()` execution versus wall-clock backpressure. A conventional kernel-socket path would deliberately obscure portions of the packet lifecycle behind socket queues, scheduler wake-ups, generic buffering, & transport-stack policies. While valuable for general-purpose applications, such mechanisms would blur the principal experimental boundaries.

"UDP" is selected for a more specific reason than the generic assertion that an unreliable transport is simply "faster" than "TCP". In the present system, **the transport is part of the experimental protocol contract**. "RFC 768" defines "UDP" as a transaction-oriented datagram service with a minimum 8-byte transport header, while "RFC 8085" characterises it as a minimal message-passing transport that avoids connection establishment / teardown & maintains little transport-layer end-system state. "TCP", conversely, deliberately provides a reliable, in-order byte stream through sequence / acknowledgment state, send & receive windows, retransmission, congestion control, & a connection state machine. Those properties are valuable when the application requires stream reliability; they are stronger than the semantics required by the present packet-oriented service paths.

The distinction is especially important for the two control loops. `temporal_payload` & `pose_payload` are not fragments of a command stream whose every historical transition must eventually be executed. They represent **absolute desired state**:

```text
"Temporal" -> { frame reference, timestamp, requested skip }
"Pose"     -> { timestamp, absolute yaw, absolute pitch, absolute zoom }
```

Accordingly, the reliability objective is **eventual convergence toward the most recent admissible state**, rather than reliable delivery of every intermediate byte or command event. `Encoder` re-presents an unresolved `temporal_skip` every `RETRY_FRAMES = 3` observations until the returned `Camera` stream reflects the requested factor. `User` analogously re-presents the same unresolved "Pose" every three completed frames until the requested stance is visible in the returning reconstructed stream. "Pose" retries preserve the original timestamp; `Decoder` ignores an already consumed timestamp & rejects older timestamps, making duplicate or delayed copies semantically harmless. A newer browser request supersedes the preceding unresolved request at `User`. The application therefore supplies the reliability property actually required by the control plane: **latest-state confirmation with duplicate / stale-state suppression**.

This distinction is more than a reformulation of head-of-line blocking. Carrying these controls through "TCP" would impose a different semantic contract: the transport would reliably preserve an ordered history of intermediate commands even when a more recent absolute state has rendered those commands obsolete. The application would still require its own coalescing / supersession logic to recover latest-state semantics. Furthermore, "TCP" exposes a byte stream rather than application message boundaries. A native `SFF` could no longer classify one command through the current deterministic `ingress port + exact payload length + protocol fields` rule without introducing a stream-framing / reassembly layer or terminating the connection. "UDP" instead maintains a one-command / one-datagram correspondence, allowing `SFF2` & `SFF3` to classify, encapsulate, validate, & decapsulate a complete control entity without transport reassembly.

The same uniformity applies to the primary path. Every native component already constructs & validates "Ethernet" / "IPv4" / "UDP" envelopes directly; raw geometry, compressed media, reconstructed points, "Temporal", & "Pose" consequently share one packet-processing model. Replacing only the vital controls with "TCP" would either require a kernel off-ramp or the introduction of an additional user-space "TCP" implementation with its own connection control blocks, queues, timers, acknowledgment processing, retransmission state, & stream parsing. Either alternative would introduce a qualitatively different software subsystem into the very data-plane experiment being measured.

The byte saving is real but is deliberately treated as a **secondary, lower-bound consequence**, not as the sole justification. For an identical application payload, the minimum "TCP" header is 20 B while the current "UDP" header is 8 B. Thus, before considering "TCP" options or acknowledgment-only traffic, each control data segment carries at least 12 additional transport bytes under "TCP"; Section 5.5 quantifies the resulting packet-size differences for the exact 16-B & 24-B project commands. A persistent "TCP" connection would of course amortise connection establishment & can employ `TCP_NODELAY`; therefore the project does **not** claim that every command would incur a fresh handshake or that "TCP" is intrinsically unsuitable for low-latency control. The choice instead follows from a tighter correspondence between the required state semantics & a datagram-oriented, directly inspectable "DPDK" path.

This architecture also avoids treating "UDP" as implicitly reliable. Data correctness is rendered observable through explicit frame / packet identifiers, point / media counters, integrity percentages, protocol field validation, redundant "End-of-Stream" ( "EOS" ) signalling, & bounded local resubmissions when a Tx ring temporarily accepts zero elements. Control correctness is established separately through returned-state confirmation, monotonic command metadata, & application-level re-presentation.

Finally, the deployment scope matters. "RFC 8085" correctly requires congestion-safe behaviour for "UDP" applications intended for the general Internet. The present implementation is instead a **closed, single-administrative-domain experimental environment** composed of six explicit local "vhost-user" / "virtio-user" adjacencies. Source production is isochronously bounded at `TARGET_FPS = 30`, local overload is directly observable through "DPDK" queue telemetry, & the workload-driven "Temporal" loop can reduce source admission. This restricted applicability is part of the specification; the current design must not be generalised into a claim that the same zero-checksum / no-general-Internet-congestion-control configuration is appropriate for unrestricted Internet deployment.

**Accordingly**, "DPDK" + "UDP" provide the project with:

```text
explicit packet ownership & lifetime
one application message = one inspectable datagram
fixed-width service-path classification without stream reassembly
in-place service-header manipulation
no native "TCP" connection-control state in the SFF data path
latest-state control semantics with application-level confirmation
observable local queue backpressure
frame-aware loss / integrity accounting
"MTU"-bounded packetisation without "IP" fragmentation
uniform transport parsing across "Main" / "Temporal" / "Pose" paths
source-level regulation rather than hidden transport flow-control policy
```

Performance therefore remains an important consequence, but the more defensible argument is architectural: **"UDP" matches the packet-level "SFC" experiment, the absolute-state control model, & the direct "DPDK" implementation while leaving reliability at the same semantic layer that defines whether a command has actually taken effect**.

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


This methodical approach is informed by the application-level architecture documented in the reference thesis upon which the present investigation builds. The reference implementation is already distributed across four containerised services — `Camera`, Flow Controller, `Encoder`, & Client — & should therefore not be inaccurately described as a single-process monolith. The relevant limitation is **coarse deployment granularity**: its `Encoder` & Client each remain internally monolithic placement blocks even though they combine operations with markedly different "CPU", "GPU", network, & "codec" requirements. The reference thesis itself identifies their decomposition into independently deployable functions as a natural next step for service orchestration.

The current repository deliberately materialises that change of paradigm. It preserves the fundamental unidirectional volumetric path & independent reverse controls, while reformulating transport, service steering, placement boundaries, & control ownership around an explicit software Data Plane:

```text
"DPDK"-native packet I / O
direct "virtio-user" / "vhost-user" adjacencies
explicit "SFC" topology without an intermediate "OVS" Data Plane
"NSH"-style "SPI" / "SI" steering
in-place geometric aggregation
stateful proxying around "SFC"-unaware applications
"GPU" projection & reconstruction
persistent "FFmpeg" / "NVENC" encoding
persistent "FFmpeg" / "NVDEC" decoding
frame-aware "MPEG-TS" attribution
workload-driven "Temporal" source regulation
independent User-originated "Pose" control applied at Decoder
asynchronous latest-frame browser delivery
per-node native telemetry
post-stream objective quality assessment
```

The architectural contrast is therefore concrete:

| Dimension | Reference Application Pipeline | Current "SFC" / "DPDK" Pipeline |
|---|---|---|
| Primary deployment graph | 4 coarse services: `Camera`, Flow Controller, `Encoder`, Client | 7 native placement units: `Camera`, `SFF1`, `SFF2`, `Encoder`, `Decoder`, `SFF3`, `User` |
| Raw upstream transport | Persistent "TCP" through `Camera -> Flow Controller -> Encoder` | Direct packet-oriented "UDP" / "DPDK" adjacencies |
| Encoded-media transport | "UDP" from `Encoder` to Client | "UDP" throughout Route 1 with explicit `SFF` proxy state |
| "Temporal" ownership | Client-originated relative control applied by a downstream Flow Controller | `Encoder`-derived absolute skip applied at `Camera` before source packetisation |
| "Pose" ownership | Client -> `Encoder`; new pose is reflected through re-projection & subsequent encoding | `User` -> `Decoder`; pose is applied during reconstruction without forcing Encoder re-projection |
| Control transport | Two persistent "TCP" feedback sockets | Two fixed-width "UDP" service paths with application-level convergence confirmation |
| Service state | Implicit in application connections / node order | Explicit `SPI` / `SI` state across aware boundaries & proxies |
| Unaware functions | Not an SFC concern | `Encoder` & `Decoder` intentionally remain "SFC"-unaware behind `SFF2` proxying |
| Placement granularity | `Encoder` & Client must be placed as whole blocks | Geometry service, encoding, decoding / reconstruction, terminal delivery, & `SFF` boundaries expose separate scheduling / placement points |

This decomposition should not be confused with a claim that adding service functions automatically reduces total compute consumption. The current primary graph exposes seven schedulable native nodes rather than four reference services — a `75 %` increase in graph-level placement units — because the objective is **orchestration granularity**, not merely process-count minimisation. The advantage is that an orchestrator can reason about measured roles separately: for example, the `SFF1` geometry stage can be placed independently from `Encoder`; `Decoder` can be separated from `User`; & `SFF` boundaries can steer different service paths without requiring the application codecs to understand the chaining protocol. The corresponding cost — additional forwarding boundaries & metadata handling — remains observable in the route telemetry rather than being treated as free.

The control placement also changes where work can be shed. In the reference design, temporal selection occurs at the Flow Controller after the Camera has already transmitted the source frame over "TCP"; even a discarded frame must be drained from that incoming stream to preserve framing. Here, `Camera` applies the returned `temporal_skip` **before packetisation**, so an admitted factor `skip = n` reduces subsequent source injection nominally to `TARGET_FPS / n`. This means the load-shedding decision can suppress raw point traffic at its origin instead of paying the upstream transport / drain cost first. The final representative runs remain at `skip = 1`, so this structural saving is implemented but not claimed as a measured overload result.

A similar shortening occurs for interactive pose changes. The reference "Pose" command reaches `Encoder`, requiring the newly requested view to traverse projection, encoding, transport, decoding, & reconstruction before it becomes visible. The current "Pose" path terminates at `Decoder`, where the latest stance is applied during reconstruction. This removes re-projection / re-encoding from the **command-to-effect dependency chain** while deliberately leaving the source / encoded representation unchanged. The current NON-QUALITY archive measures this path directly through `reference_cmd_ms`, `cmd_apply_ms`, `Decoder` `pose_control_ms`, & browser `cmd_photon_ms`.

The removal of the former "OVS"-"DPDK" virtual-switch intermediary constitutes a separate topology simplification within the current project lineage. Adjacent microservices communicate directly through project-defined Unix-domain "vhost-user" sockets, eliminating the dedicated virtual-switch "PMD" role & releasing one logical core that had previously been reserved exclusively for switching. On the present 8-logical-"CPU" host, this converts `1 / 8 = 12.5 %` of logical scheduling capacity from switch-only reservation into application-available placement capacity. It is a **resource reallocation**, not evidence that the whole application now consumes 12.5 % less "CPU" time.

At the measurement level, the repository deliberately preserves the experimental principle adopted by the reference pipeline: frame-associated indicators are resolved during execution without synchronous native CSV writes inside the real-time path, retained in memory, & serialised only after the terminal condition. The present "SFC" implementation extends that baseline with service-state, protocol-integrity, local "DPDK" backpressure, reverse-control, browser-acknowledgment, & post-stream quality observability while keeping the authoritative application identity anchored to `frame_id`.

Section 21.15 provides the direct numerical comparison supported by the supplied reference material. Its interpretation is deliberately conservative: the evidence supports a materially lower median end-to-end latency & lower selected application-stage costs while retaining the same 300-frame / 30-fps operating objective & a closely comparable reconstruction-fidelity regime. It does **not** support the stronger statement that every per-node residency is lower, that aggregate "CPU" utilisation has already been reduced by a measured percentage, or that every objective quality metric is numerically invariant. Such claims would require a controlled A / B campaign on identical hardware, software boundaries, instrumentation, & metric implementations.

### 1.4 Guidelines & "Codec" Scope

The presentation of the work should make explicit that persistent "FFmpeg" / hardware-"codec" execution is treated as a stateful subsystem, not as a per-frame subprocess. `Encoder` & `Decoder` are therefore stabilised through readiness files, private pre-roll frames, terminal post-roll coordination, & repeated "EOS" propagation. These mechanisms are part of the experimental design because they remove startup / drain ambiguity from the 300 application frames & allow the native telemetry to remain tied to the real service frames only.

Two distinctions remain critical for accurate interpretation of this work.

Firstly, the project integrates the **"SFC" concepts** of "Service Path Identifier" ( "SPI" ) & "Service Index" ( "SI" ), alongside an experimental "MD-Type-2"-like context layout. Nevertheless, the wire representation constitutes a closed project protocol & is appropriately characterised as **"NSH"-inspired**, rather than asserting generic "RFC 8300" interoperability.

Secondly, `Encoder` does **not** implement "MPEG" "V-PCC" or "G-PCC". It constructs a custom 6-view "Geometry" / "Texture" / "Occupancy" "Atlas", employing "HEVC" ( `hevc_nvenc` ) as the video compression engine for this representation. The resulting bitrate & latency metrics characterise the specific projection-&-video path & should not be presented as standards-compliant coding benchmarks.

---

## 🧩 2. Architectural Roles & Why Each Node Matters

| Device | Importance |
|---|---|
| `Camera` | Establishes the absolute source timeline, reads prepared fixed-width frames, serialises coordinates into network byte order, packetises below the "MTU", & applies the most recent `temporal_skip` before frame injection. |
| `SFF1` | Demonstrates the in-path computation principle. It evaluates progressive centroid / extent / bounding-box information while forwarding & computes the exact final `max_r` at frame completion, exporting geometry through the experimental service context. |
| `SFF2` | Separates service-path state from unaware application functions. Its four ports implement all three primary-route transitions, plus the reverse "Temporal" & "Pose" paths, while preserving `SPI` / `SI` state around `Encoder` & `Decoder`. |
| `Encoder` | Reconstructs complete frames, consumes `SFF1` geometry when valid, executes the "CUDA" six-view projection, feeds a persistent pre-rolled "NVENC" process, attributes asynchronous "MPEG-TS" output, & regulates source admission from measured workload. |
| `Decoder` | Receives plain `cam_hdr + enc_hdr + MPEG-TS`, feeds a persistent hardware decoder, reconstructs the projected representation into a point cloud, applies the current pose, & emits a packet-sequenced `dec_hdr + point_tx` stream. |
| `SFF3` | Terminates the primary aware chain before the end user & originates the reverse "Pose" service chain. It strips `SPI 100 / SI 253` on the data path & adds `SPI 300 / SI 255` to validated `User` commands. |
| `User` | Validates final point packets, reassembles reconstructed geometries, publishes only completed snapshots to the local Web bridge, tracks pose requests, gathers end-to-end / browser acknowledgments, & coordinates quality capture. |
| "Python" bridge / Viewer | Implements an asynchronous, latest-frame-only presentation frontier. A one-element peer queue & a one-frame-in-flight acknowledgment gate prevent progressive browser backlog from feeding back into the native `User` data path. |
| `Gauge` | Executes only after quality-stream completion, recovering objective geometry metrics without competing with the real-time network path. |

The virtual topology is intentionally direct:

```
Camera <-> SFF1 <-> SFF2 <-> Encoder
                     | |
                     | +---- Decoder
                     |        
                     +------ SFF3 <-> User
```

The exact direct sockets are:

```
Camera <-> SFF1    /tmp/sfc-cam-sff1
SFF1   <-> SFF2    /tmp/sfc-sff1-sff2
SFF2   <-> Encoder /tmp/sfc-sff2-enc
SFF2   <-> Decoder /tmp/sfc-sff2-dec
SFF2   <-> SFF3    /tmp/sfc-sff2-sff3
SFF3   <-> User    /tmp/sfc-sff3-usr
```

There is therefore no additional virtual-switch processing tier. `SFF1`, `SFF2`, & `SFF3` are the sole nodes that interpret service-chain semantics, while `Encoder` & `Decoder` remain intentionally "SFC"-unaware.

---

## 🔗 3. Service-Chain Semantics

### 3.1 "Main" Service Path

The project defines:

```
MAIN_SPI = 100
```

Primary service state evolves as:

```
SFF1 emits aware state                         : "SPI 100", "SI 255" + geometry context
SFF2 captures state & removes service envelope : Encoder receives plain "UDP" + geo_agg_hdr
Encoder returns plain compressed traffic       : proxy state advances "SI 255 -> 254"
SFF2 forwards plain application traffic        : Decoder remains "SFC"-unaware
Decoder returns reconstructed points           : proxy state advances "SI 254 -> 253"
SFF2 re-imposes base service envelope          : "SPI 100", "SI 253"
SFF3 validates & removes base envelope         : User receives plain "UDP" + dec_hdr + points
```

The entire route is operational:

```
Camera
  -> SFF1 ( "GAC", aware boundary )
  -> SFF2 ( Route 0, proxy capture / decapsulation )
  -> Encoder ( unaware )
  -> SFF2 ( Route 1, proxy transition )
  -> Decoder ( unaware )
  -> SFF2 ( Route 2, proxy transition / re-encapsulation )
  -> SFF3 ( aware boundary / decapsulation )
  -> User
```

Route-specific completion, byte accounting, protocol validation, & Tx acceptance telemetry are retained independently by `SFF2` for all three transitions.

### 3.2 "Temporal" Service Path


"Temporal" adaptation utilises:

```
TEMPORAL_SPI = 200
TEMPORAL_SI  = 255
```

`Encoder` generates the request from its internal workload model. The current packed control structure is:

```
frame_id  : uint32
timestamp : uint64
skip      : uint16
padding   : uint16
------------------
Total     : 16 B
```

The path is:

```
Encoder
  -> plain 16-B "UDP" payload
  -> SFF2 classifies + imposes "SPI 200 / SI 255"
  -> SFF1 validates + removes the service envelope
  -> Camera validates the exact 16-B datagram & accepts a non-older frame-associated decision
```

The command carries the **absolute requested factor**, rather than an increment / decrement event. `Camera` normalises an invalid zero factor back to `1` & accepts the request only when its `frame_id` is not older than the most recently accepted control reference. Once adopted, source admission is evaluated before point packetisation according to:

```
selected( frame_id ) = ( ( frame_id - 1 ) mod skip ) == 0
FPS                  = TARGET_FPS / skip
```

Consequently, temporal reduction occurs at the earliest useful frontier: skipped source frames do not consume point packetisation, `Camera` Tx descriptors, or any subsequent primary-route service capacity. For example, if a controlled overload experiment requests `skip = 2`, the nominal admitted source rate becomes `15 frames/s`; `skip = 3` yields `10 frames/s`. These are deterministic consequences of the selection rule, not measured outcomes of the final `skip = 1` archives.

Reliability is expressed as returned-state convergence. If `workload_controller.requested_skip != active_skip`, `Encoder` suppresses further overload / recovery transitions & re-dispatches the desired factor after `RETRY_FRAMES = 3` observations. The request is considered semantically reflected only when subsequent `cam_hdr.temporal_skip` values return the requested state. A lost control datagram can therefore delay adoption, but cannot permanently orphan the requested state while the pipeline continues to produce complete observations. This mechanism is intentionally distinct from transport retransmission: it verifies the **application effect**, not merely delivery to a socket.

### 3.3 "Pose" Service Path


"Pose" control remains independent from temporal regulation:

```
POSE_SPI = 300
POSE_SI  = 255
```

The packed command is:

```
timestamp : uint64
yaw       : uint32 # network-order float bit pattern
pitch     : uint32 # ...
zoom      : uint32 # ...
padding   : uint32
------------------
Total     : 24 B
```

The operational route is:

```
Browser
  -> Python bridge shared control map
  -> User ( plain "UDP" )
  -> SFF3 ( classify + impose "SPI 300 / SI 255" )
  -> SFF2 ( validate + decapsulate )
  -> Decoder ( plain "UDP" command )
```

The payload represents an **absolute stance**, rather than a relative rotation event. `User` timestamps the first dispatch, stores the requested `yaw`, `pitch`, & `zoom`, & re-presents the same timestamped state every `RETRY_FRAMES = 3` completed frames while it remains unresolved. `Decoder` accepts finite positive-zoom commands, discards an already consumed timestamp, rejects any timestamp older than the active stance, snapshots the latest admissible state before reconstruction, & applies that state on the "GPU". This combination makes retransmitted duplicates idempotent & prevents a delayed older command from rolling the view backward.

The reconstructed frame returns the pose actually applied by `Decoder`. `User` therefore closes the semantic loop by matching the returned `yaw` / `pitch` / `zoom` against the command record, rather than assuming success after local Tx acceptance. A newer browser command replaces the currently active unresolved request, reflecting the latest-state semantics expected from direct manipulation.

The browser remains fire-&-forget from an interaction standpoint: command issuance does not synchronously block upon an acknowledgment. A distinct browser acknowledgment is retained only for telemetry & one-frame-in-flight Web scheduling. This means the reliability mechanism does not insert a request / response stall into the input path; it operates asynchronously through returned state & optional re-presentation.

### 3.4 Protocol Clarification


The architecture employs an 8-byte `nsh_hdr`, the "SPI" / "SI" paradigm, a "TTL" field, & a project-defined "MD-Type-2"-like geometric context. The implementation is accurately described as **experimental / "NSH"-inspired**, rather than as a universally interoperable "RFC 8300" implementation. "RFC 7665" is relevant at the architectural level because it defines SFFs, SFC-aware / unaware functions, proxies, classification, metadata exchange, & topology-independent service paths; "RFC 8300" motivates the `SPI` / `SI` progression & proxy behaviour. Neither RFC requires the project to employ "UDP" as its transport, so transport uniformity is an implementation choice rather than a standards-compliance claim.

The service-plane constants are fixed as part of the experiment:

```text
MAIN_SPI                   = 100
MAIN_SI_SFF1               = 255
MAIN_SI_ENCODER            = 254
MAIN_SI_DECODER            = 253

TEMPORAL_SPI               = 200
TEMPORAL_SI                = 255
POSE_SPI                   = 300
POSE_SI                    = 255

DEFAULT_TTL                = 63
MD_CLASS_EXPERIMENTAL      = 0xFFF6
MD_TYPE_GEOMETRY           = 0x01
MD_TYPE_2                  = 0x02
NEXT_PROTOCOL_EXPERIMENT_1 = 0xFE
```

The corresponding protocol contract can be summarised without relying upon implicit application behaviour:

| Family | Classification / Entry | Service State | Fixed Application Entity | Terminal Acceptance Rule |
|---|---|---|---|---|
| "Main" | `Camera` point datagrams enter through `SFF1`; subsequent proxy transitions are selected by ingress port & preserved primary state | `SPI 100`, `SI 255 -> 254 -> 253` | `cam_hdr + points`, then `cam_hdr + enc_hdr + MPEG-TS`, then `dec_hdr + points` | Exact frame / packet populations, route-specific metadata, & expected `SPI` / `SI` at aware boundaries |
| "Temporal" | Exact 16-B plain payload from `Encoder`-facing port | `SPI 200 / SI 255` between `SFF2` & `SFF1` | `temporal_payload` | Nonzero normalised absolute skip, non-older frame reference, returned `cam_hdr.temporal_skip` confirmation |
| "Pose" | Exact 24-B plain payload at `SFF3`, then exact base-aware payload at `SFF2` | `SPI 300 / SI 255` between `SFF3` & `SFF2` | `pose_payload` | Finite stance, positive zoom, monotonic timestamp, returned pose matching at `User` |

This explicit contract is central to research validity: route selection, packet shape, accepted state transition, & completion criteria are fixed in code rather than inferred retrospectively from observed traffic.

The current address contract is:

| Adjacency / Endpoint | "IPv4" Address | "UDP" Port |
|---|---|---:|
| `Camera` | `10.0.1.1` | `5001` |
| `SFF1` `Camera`-facing endpoint | `10.0.1.254` | `6633` |
| `SFF1` `SFF2`-facing endpoint | `10.0.2.1` | `6633` |
| `SFF2` `SFF1`-facing endpoint | `10.0.2.2` | `6633` |
| `Encoder` | `10.0.3.1` | `7001` |
| `SFF2` `Encoder`-facing endpoint | `10.0.3.254` | `6633` |
| `Decoder` | `10.0.4.1` | `8001` |
| `SFF2` `Decoder`-facing endpoint | `10.0.4.254` | `6633` |
| `SFF3` `SFF2`-facing endpoint | `10.0.5.1` | `6633` |
| `SFF2` `SFF3`-facing endpoint | `10.0.5.2` | `6633` |
| `User` | `10.0.6.1` | `9001` |
| `SFF3` `User`-facing endpoint | `10.0.6.254` | `6633` |

These allocations form a closed testbed contract rather than a dynamically routed deployment. Native nodes construct & validate the corresponding "Ethernet" / "IPv4" / "UDP" envelopes directly through the six Unix-domain "DPDK" adjacencies.

When an "IPv4" header is generated or rewritten, its checksum is cleared & recomputed via `rte_ipv4_cksum()`. The project deliberately maintains a zero "UDP" checksum for the present "IPv4" testbed. "RFC 768" permits a transmitted zero checksum to indicate that no checksum was generated, but "RFC 8085" recommends checksum protection for general use; the current choice is therefore a **restricted experimental convention**, compensated by a closed local topology plus explicit application / protocol integrity validation, & must not be generalised to future "IPv6" or unrestricted Internet operation.

---

## 📦 4. Data Representation & Packet Formats

### 4.1 Endianness & Portability

Offline ( `.bin` ) & live network representations fulfil distinct responsibilities. `converter.py` constructs contiguous 16-byte host records using Little-Endian `float32` coordinates; network-facing components reinterpret these floating-point bit patterns as 32-bit words & serialise them in network byte order. All integer protocol fields are likewise explicitly converted at their boundaries.

Consequently, the storage artefact is a documented local representation, whereas the live "DPDK" path is an explicit network representation. `Decoder`, `SFF3`, & `User` preserve the identical coordinate convention when reconstructed points are returned downstream.

### 4.2 Common Structures

| Structure | Size | Function |
|---|---:|---|
| `point_tx` | `16 B` | Network point record containing `x`, `y`, `z`, "RGB", & one padding byte |
| `host_point` | `16 B` | Host / shared-memory point record with native `float` coordinates & "RGB" |
| `cam_hdr` | `40 B` | Frame identity, packet sequence, `Camera` timestamp, source pose, temporal skip, original-point count, & packet population |
| `nsh_hdr` | `8 B` | Experimental service-chain base carrying "SPI", "SI", "TTL", metadata type, & next-protocol information |
| `nsh_md2_ctx_hdr` | `4 B` | Project geometric-context descriptor |
| `geo_agg_hdr` | `44 B` | Centroid, extent, bounding-box centre, `max_r`, & active-point count |
| `enc_hdr` | `48 B` | Encoded-media packet identity & reconstruction parameters |
| `dec_hdr` | `52 B` | Reconstructed-frame identity, sequence, original / arrived / eroded / valid populations, pose, temporal skip, & packet population |
| `temporal_payload` | `16 B` | Frame / timestamp-associated temporal request |
| `pose_payload` | `24 B` | Timestamped `yaw`, `pitch`, `zoom`, & alignment padding |
| `web_hdr` | `72 B` | Shared-memory snapshot metadata consumed by the asynchronous Web bridge |
| `web_ctrl` | `56 B` | Shared command + browser-acknowledgment exchange structure |
| `quality_hdr` | `8 B` | quality-capture record identity & reconstructed point count |

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

The deterministic 16-byte width is shared by the offline Converter, live point packets, reconstructed host buffers, quality capture, & browser payload extraction.

### 4.4 Camera Header

The 40-byte `cam_hdr` carries:

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

The `Camera` introduces the neutral source pose (`0`, `0`, `1`). `User`-originated pose changes are deliberately applied at `Decoder`, rather than altering the source / `Encoder` projection state.

### 4.5 Geometric Metadata Produced by SFF1

The "GAC" exports progressive centroid, extent, bounding-box centre, exact final `max_r`, & `active_point_count`. For the observed prefix `P_N`:

```
C_N = ( 1 / N ) * sum_{ p in P_N }( p )
E_N = p_max,N - p_min,N
B_N = ( p_min,N + p_max,N ) / 2
```

At frame completion:

```
max_r = max_{ p in P_frame } || p - C_final ||_2
```

`Encoder` validates the final active-point count before accepting this offloaded geometry.

### 4.6 Encoder Structure

The 48-byte `enc_hdr` accompanies the compressed stream:

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

`packet_id` resets for each attributed encoded application frame. The remaining parameters allow `Decoder` to reconstruct the six-view projection using the identical geometric convention employed by `Encoder`.

### 4.7 Decoder Output Structure

The 52-byte `dec_hdr` restores frame semantics after decoding / reconstruction:

```
frame_id
sequence_number
timestamp
yaw
pitch
zoom
temporal_skip
padding
original_points
arrived_points
eroded_points
valid_points
points_in_packet
```

`timestamp` retains the original `Camera` timeline. `arrived_points`, `eroded_points`, & `valid_points` distinguish the progressive reconstruction stages, while the pose fields report the stance actually applied by `Decoder`. `SFF2` & `SFF3` therefore need not infer reconstruction population from packet counts alone.

### 4.8 User Shared-Memory & Quality Records

`User` reconstructs complete frames into the shared point region following a 72-byte `web_hdr`. An odd / even sequence protocol provides a lock-free publication marker: odd values indicate an in-progress frame, while an even value identifies a completed stable snapshot. In `QUALITY_CAPTURE = 1`, Web publication & sequence mutation are disabled, as no browser process participates in the experiment.

The 56-byte `web_ctrl` region carries command sequence, command identifier, command type, requested pose, acknowledgment sequence, rendered frame, rendered command, & measured browser `Command-to-Photon` latency.

Quality capture serialises a stream of:

```
quality_hdr { frame_id, point_count } followed by point_count * host_point
```

This representation permits `gauge.py` to index reconstructed frames directly without introducing a second conversion format.

---

## 📐 5. "MTU"-Aware Packet Design

The system targets the conventional:

```
"IPv4" "MTU" = 1500 B
```

All application datagrams remain below that threshold. "Ethernet" framing is reported separately because the "MTU" excludes the 14-byte "Ethernet" header.

### 5.1 Camera -> SFF1

```
POINTS_PER_PACKET = 80
POINT_SIZE_BYTES  = 16
Point payload     = 1280 B
```

```
"IPv4"                     20 B
"UDP"                       8 B
cam_hdr                    40 B
80 * point_tx            1280 B
-------------------------------
"IPv4" datagram          1348 B
"Eth" frame              1362 B
"IP"-"MTU" margin         152 B
```

### 5.2 SFF1 -> SFF2 -> Encoder

Aware `SFF1` output:

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
```

Route 0 removes the service base / context while preserving `geo_agg_hdr` for the unaware `Encoder`:

```
20 + 8 + 44 + 40 + 1280 = 1392 B
```

### 5.3 Encoder -> SFF2 -> Decoder

```
TS_PACKET_SIZE   = 188 B
MTU_PAYLOAD_SIZE = 7 * 188 = 1316 B
```

```
"IPv4"                     20 B
"UDP"                       8 B
cam_hdr                    40 B
enc_hdr                    48 B
7 * "MPEG-TS"            1316 B
-------------------------------
"IPv4" datagram          1432 B
"IP"-"MTU" margin          68 B
```

### 5.4 Decoder -> SFF2 -> SFF3 -> User

`Decoder` emits a plain packet containing a 52-byte reconstruction header:

```
"IPv4"                      20 B
"UDP"                        8 B
dec_hdr                     52 B
80 * point_tx             1280 B
--------------------------------
Decoder -> SFF2           1360 B
```

Route 2 re-imposes the 8-byte service base:

```
SFF2 -> SFF3 = 1360 + 8 = 1368 B
```

`SFF3` removes that base before `User`, returning the datagram to `1360 B`. The complete reconstructed-point route therefore retains at least `132 B` of "IPv4" "MTU" headroom at its largest aware boundary.

### 5.5 "Temporal" & "Pose" Packets


"Temporal":

```
Encoder -> SFF2 plain   = 20 + 8 + 16     = 44 B
SFF2    -> SFF1 aware   = 20 + 8 + 8 + 16 = 52 B
SFF1    -> Camera plain = 44 B
```

"Pose":

```
User -> SFF3 plain    = 20 + 8 + 24     = 52 B
SFF3 -> SFF2 aware    = 20 + 8 + 8 + 24 = 60 B
SFF2 -> Decoder plain = 52 B
```

Every command therefore fits in one unfragmented datagram & preserves a direct one-message / one-packet relation at the application boundary. This property is used by the native classifiers: no command stream has to be reconstructed before the corresponding service path can be selected.

For the exact current payloads, the **minimum-header** comparison against an otherwise identical "IPv4" / "TCP" carrier is:

| Control Entity | Current "UDP" L3 Packet | Hypothetical Minimum-"TCP" L3 Packet | Current Byte Reduction |
|---|---:|---:|---:|
| "Temporal" plain | `20 + 8 + 16 = 44 B` | `20 + 20 + 16 = 56 B` | `21.4 %` |
| "Temporal" aware | `20 + 8 + 8 + 16 = 52 B` | `20 + 20 + 8 + 16 = 64 B` | `18.8 %` |
| "Pose" plain | `20 + 8 + 24 = 52 B` | `20 + 20 + 24 = 64 B` | `18.8 %` |
| "Pose" aware | `20 + 8 + 8 + 24 = 60 B` | `20 + 20 + 8 + 24 = 72 B` | `16.7 %` |

Including the 14-B "Ethernet" header, the corresponding reductions are approximately `17.1 %`, `15.4 %`, `15.4 %`, & `14.0 %`. These percentages are deliberately conservative: they assume the minimum 20-B "TCP" header, no "TCP" options, & no acknowledgment-only traffic. Conversely, they do **not** count a three-way handshake per command, because a sensible comparison would employ a persistent "TCP" control channel as the reference architecture does. The table therefore isolates the transport-envelope difference without constructing an artificially weak "TCP" baseline.

The principal benefit nevertheless remains semantic & architectural rather than bandwidth-driven. At the measured command rates, 12 transport bytes are not the dominant system cost. The more consequential property is that the command remains a self-delimiting object which `SFF2` / `SFF3` can classify immediately, while duplicate / loss handling is tied to the returned application state rather than to reliable byte delivery.

The two command families are structurally independent & carry their own timing / state information. "Temporal" uses frame-associated state confirmation; "Pose" preserves one command timestamp across retries & relies upon `Decoder` monotonicity. Consequently, both can remain fire-&-forget at dispatch time without making control correctness dependent upon a hidden transport acknowledgment.

### 5.6 "Virtio" Queue Size & "DPDK" Rx / Tx Dimensioning

The final native configuration deliberately normalises every `rte_eth_rx_queue_setup()` & `rte_eth_tx_queue_setup()` call to:

```
4096 descriptors
```

The corresponding `User` `virtio-user` attachment also employs:

```
queue_size = 4096
```

This final value is a measured experimental condition rather than an assertion that larger queues intrinsically improve performance. Descriptor depth, `virtqueue` capacity, core affinity, bounded zero-accept retries, & consumer service time form one joint buffering / scheduling condition. Prior queue-depth experiments are therefore not silently mixed with the final result set.

Each native node retains:

```
NUM_MBUFS        = 16383
MBUF_CACHE_SIZE  = 256
BURST_SIZE       = 32
MAX_ZERO_ACCEPTS = 2048
```

A zero return from `rte_eth_tx_burst()` represents **local Tx-ring acceptance pressure**, not a network-level "UDP" retransmission. Native telemetry consequently records zero accepts, partial accepts, resubmission calls, & re-presented packet counts separately from frame-completion semantics.

`MAX_ZERO_ACCEPTS` bounds the **consecutive** zero-accept streak associated with a pending burst rather than the cumulative `tx_zero_accepts` value of an entire frame. Any positive acceptance resets the local retry streak; consequently, a frame may legitimately report more than `2048` total zero-accept events while still reaching `tx_complete = 1`. This distinction is essential when interpreting the comparatively large `Camera` counters reported by the final archives.

---

## 📷 6. Camera — Scheduling, Warm-Mode, "Temporal" Selection, & Telemetry

### 6.1 Role

The `Camera` functions as the exclusive node originating the volumetric schedule. It ingests the pre-converted "Loot" sequence, allocates source frame IDs, serialises coordinates for network transmission, packetises each point cloud, timestamps the element, & submits "DPDK" bursts to `SFF1`. Consequently, its contribution is simultaneously functional & experimental. Every downstream timing quantity is ultimately conditioned by the source cadence, burst structure, cache mode, & residency policy established at this component.

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
| `CACHE_MODE_MIDDLE`              | Before streaming  | Inside frame loop | Storage-aware experiment; disk / page-cache access remains visible within `Camera` residency      |
| `CACHE_MODE_WORST`               | Inside frame loop | Inside frame loop | Deliberately pessimistic mode incorporating allocation & file read within the frame path        |

The current source configuration selects:

```text
CACHE_MODE = CACHE_MODE_MIDDLE
WARM_MODE  = WARM_MODE_ENABLED
```

`WARM_MODE_ENABLED` maps & locks source documents prior to the measured sequence, ensuring the timed `fread()` path retains standard file-read semantics while operating across a resident file-backed working set. The resulting `disk_io_ms` should be interpreted as **timed buffered acquisition from a warmed condition**, rather than a direct measurement of cold physical-storage latency.

Results obtained under varying settings constitute distinct experimental conditions & must not be conflated within identical performance claims.

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

The `Camera` timestamp is generated immediately prior to the packet-transmission loop & propagates unchanged across all packets corresponding to the same frame.

Coordinates transition from the prepared little-endian host representation to network-order "IEEE-754" bit patterns prior to transmission. "RGB" values & the explicit padding byte remain byte-valued fields.

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

Both selected & skipped frames advance against the identical absolute source timeline. This mechanism prevents temporal adaptation from redefining the session clock, thereby preserving meaningful frame-ID-based scheduling & downstream jitter calculations.

Consequently, the current design omits a separate `PACING_MODE`. Source timing is governed entirely by the absolute target schedule, whereas local Tx-ring pressure is exposed via retry telemetry rather than being obscured behind a supplementary pacing heuristic.

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

The validated run exhibits numerous zero-accept attempts due to the emission of approximately ten thousand point packets per frame; nevertheless, every frame remains complete & `mbuf_starvation = 0`. The counters thereby provide robust evidence that backpressure existed without translating into application-visible loss during this experiment.

### 6.6 Camera Telemetry — Complete Semantics

The final `Camera` exports **30 fields**. The schema distinguishes source selection, control application, packet population, logical / network rates, file acquisition, serialization, active Tx execution, full source residence, & local queue-acceptance pressure:

```text
frame_id;selected;tx_complete;current_skip;last_control_frame;temporal_control_ms;camera_send_timestamp;tx_start_timestamp;tx_points;tx_packets;payload_bytes;reference_size_bytes;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;disk_io_ms;serialization_ms;tx_duration_ms;active_tx_ms;active_process_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```

| Field / Group | Exact Semantics |
|---|---|
| `frame_id` | Original source identifier from the 300-frame sequence. |
| `selected` | `1` when the frame survives the current temporal admission rule; the final runs retain all 300 frames because `current_skip = 1`. |
| `tx_complete` | `1` only when all selected point packets are successfully accepted by the local "DPDK" Tx path. |
| `current_skip` | "Temporal" factor applied when deciding whether this source frame is admitted. |
| `last_control_frame`, `temporal_control_ms` | Last accepted control reference & measured local control-handling delay. |
| `camera_send_timestamp` | Source timing anchor propagated throughout the chain. |
| `tx_start_timestamp` | Local start of packet submission for the selected frame. |
| `tx_points`, `tx_packets`, `payload_bytes` | Successfully accepted point population, datagram count, & point bytes. |
| `reference_size_bytes` | Logical frame size used by the reference-throughput calculations. |
| `internal_throughput_mbs`, `reference_throughput_mbs` | Local logical-source rates under the measured / reference boundaries. |
| `logical_bitrate_mbps`, `network_bitrate_mbps`, `reference_bitrate_mbps` | Application, protocol-inclusive, & reference bitrate formulations at the current effective frame rate. |
| `disk_io_ms` | Per-frame `fread()` interval for `CACHE_MODE_MIDDLE`. |
| `serialization_ms` | Host-to-network point conversion / packet preparation cost. |
| `tx_duration_ms` | Wall-clock span across frame Tx submission, including local retry waits. |
| `active_tx_ms` | Time actively spent inside Tx submission calls, excluding deliberate pause / retry gaps. |
| `active_process_ms` | Source work attributed to disk read + serialization + active Tx execution. |
| `total_residency_ms` | Full selected-frame `Camera` residence from local processing start to accepted terminal packet. |
| `node_efficiency_pct`, `reference_efficiency_pct` | Active-work ratios against measured / reference residence definitions. |
| `tx_zero_accepts` | Number of local Tx calls accepting no packet. |
| `tx_partial_accepts` | Number of local Tx calls accepting fewer packets than requested. |
| `tx_resubmit_calls`, `tx_resubmitted_packets` | Local re-presentation operations & requested packet-attempt population. These are not "UDP" retransmissions. |
| `mbuf_starvation` | Packet-buffer allocation failures. The final representative runs record `0`. |

The distinction between `tx_duration_ms` & `active_tx_ms` remains essential. A frame can spend additional wall-clock time waiting for local descriptor availability without the "CPU" simultaneously executing inside `rte_eth_tx_burst()`.

### 6.7 "End-of-Stream" Behaviour

Following the configured sequence, the `Camera` emits recurrent `END_OF_STREAM` control packets containing the `Camera` header & devoid of point payload. This redundancy ensures the terminal condition remains resilient to transient local queue behaviour within the experimental environment.

The "EOS" marker constitutes a protocol event, rather than a supplementary source frame, & is excluded from the 300-frame telemetry table.

---

## 🧠 7. SFF1 — "Geometry-Aware Classifier" ( "GAC" ) & In-Path Aggregation

### 7.1 Role

`SFF1` operates as the "Geometry-Aware Classifier" ( "GAC" ) within the current architecture. Its fundamental purpose is to demonstrate that actionable frame geometry can be derived **while point packets actively traverse the service path**, rather than reconstructing identical statistics initially inside the `Encoder`.

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

The "GAC" is purposefully designed to exceed the capabilities of a mere forwarding label while remaining substantially narrower than a full-fledged application processor.

### 7.2 "Temporal" Control Is Relayed, Not Decided, by SFF1

The current `SFF1` iteration does **not** execute source-frame temporal filtering.

"Temporal" adaptation is orchestrated by the `Encoder` & enacted by the `Camera`. `SFF1` functions solely as the terminal service-chain-aware relay along the reverse "Temporal" trajectory:

```text
SFF2 -> SFF1   : "NSH"-encapsulated temporal_payload
SFF1           : validates "SPI" = 200 / "SI" = 255
SFF1           : strips the service-chain envelope
SFF1 -> Camera : plain "UDP" temporal_payload
```

The `current_skip` observed by `SFF1` on the primary data path is, therefore, the factor already adopted by the `Camera` & encapsulated within `cam_hdr`. It serves telemetry & effective-rate interpretation purposes, rather than acting as a secondary local frame-drop determinant.

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

These operations permit updating upon each point's arrival, naturally aligning with the computational profile of a data-plane-oriented classifier.

The exact farthest-point radius presents a distinct mathematical dependency:

```text
max_r = max_i || p_i - C_final ||_2
```

Since `C_final` remains indeterminate until the final point is ascertained, exact `max_r` cannot be resolved universally from the initial packet without either revisiting preceding points or offloading the computation upstream. Consequently, the current implementation records only `XYZ` coordinates within a preallocated workspace & executes a secondary radius pass upon frame completion.

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

This constitutes the central in-path transformation: service metadata is appended directly to the packet during forwarding, eschewing the creation of a disparate frame-level side channel.

### 7.5 Frame-Global Boundary Constraint

Two distinct definitions of a geometric "boundary" exist within the current implementation.

Axis-aligned extrema are packet-progressive:

```text
min_x / max_x
min_y / max_y
min_z / max_z
```

& consequently attain increasing accuracy as packets arrive.

Conversely, a centroid-dependent radial boundary cannot achieve finality until the centroid itself is finalised. Similarly, establishing a boundary contingent upon an arbitrary future user pose necessitates the prior availability of said pose. No packet format inherently resolves this mathematical dependency.

Therefore, the project deliberately avoids artificial preprocessing solutions at the `Camera` or via an offline converter. Such solutions could transmit final geometry prematurely but would negate the fundamental research objective of evaluating in-place execution at the service function **on the data path**. The current design upholds this objective, rendering the unavoidable frame-global step explicit.

### 7.6 SFF1 Telemetry — Complete Semantics

The final `SFF1` exports **42 fields**, combining frame-integrity, in-path geometry, timing, protocol validation, & Tx-backpressure observability:

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;geometry_aggregation_ms;max_r_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;camera_node_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

| Field / Group | Exact Semantics |
|---|---|
| `frame_id`, `current_skip` | Original `Camera` identity & reflected temporal factor. |
| `rx_complete`, `tx_complete` | Exact receive / forward completion predicates derived from frame sequence & point population. |
| `camera_send_timestamp`, `recv_start_timestamp`, `node_exit_timestamp` | Source, first-arrival, & final-forwarding timing anchors. |
| `original_points`, `rx_points`, `tx_points` | Declared, received unique, & successfully forwarded point populations. |
| `rx_packets`, `tx_packets` | Valid received / accepted packet counts. |
| `payload_bytes`, `reference_size_bytes` | Point payload & comparison baseline sizes. |
| `data_integrity_pct` | `100 * rx_points / original_points` for a populated frame. |
| throughput / bitrate fields | Distinguish local receive-span throughput, logical frame rate, protocol-inclusive rate, & reference formulation. |
| `geometry_aggregation_ms` | Accumulated progressive sum / extrema / centroid / extent / bounding-box work. |
| `max_r_ms` | Exact frame-completing radius evaluation after the final centroid becomes available. |
| `tx_duration_ms`, `active_tx_ms` | Wall-clock egress span versus active Tx-call execution. |
| `active_process_ms` | Geometry + exact-radius + active-forwarding work. |
| `cycle_ms`, `header_wait_ms` | Per-frame service-loop / initial-header waiting observability. |
| `total_residency_ms` | First valid input packet to accepted final output packet. |
| `node_efficiency_pct`, `reference_efficiency_pct` | Active-work ratios under measured / reference boundaries. |
| `camera_node_ms` | `Camera` source timestamp to first `SFF1` arrival. |
| `schedule_delay_ms`, `instant_jitter_ms`, `desynced_jitter_ms` | Source-cadence drift & frame-gap-aware timing variability. |
| `eth_errors`, `ipv4_errors`, `udp_errors`, `nsh_errors` | Protocol-validation failures. All remain `0` in the final representative runs. |
| Tx acceptance counters | Local zero / partial accepts & resubmission activity, independent from "UDP" semantics. |

`data_integrity_pct = 100 %` in all 300 final rows of both archived modes, while the geometry fields remain directly observable as the computation deliberately moved into the Data Plane.

---

## 🔀 8. SFF2 — Multi-Port "NSH" Proxy & Service-Path Steering

### 8.1 Role & Ports

`SFF2` is the central four-port service-path proxy. Its interfaces are:

```
PORT_SFF1    = 0
PORT_ENCODER = 1
PORT_DECODER = 2
PORT_SFF3    = 3
```

The node does not perform projection, decoding, or rendering. Its responsibility is to validate packet envelopes, preserve service state across unaware functions, classify reverse controls, rewrite the local network envelope, & maintain exact route-specific accounting.

### 8.2 Implemented Primary Routing

Three primary route identifiers are active:

```
ROUTE_SFF1_ENCODER    = 0
ROUTE_ENCODER_DECODER = 1
ROUTE_DECODER_SFF3    = 2
```

Their semantics are:

```
Route 0 : SFF1    -> SFF2 -> Encoder
Route 1 : Encoder -> SFF2 -> Decoder
Route 2 : Decoder -> SFF2 -> SFF3
```

Route 0 receives `SPI 100 / SI 255` with the geometric context, captures proxy state, removes service metadata, & preserves `geo_agg_hdr` as application-visible information. Route 1 receives plain encoded media, validates the retained context, advances `SI 255 -> 254`, & forwards a plain packet to `Decoder`. Route 2 receives a plain reconstructed-point packet, advances `SI 254 -> 253`, rebuilds the base service envelope, & forwards `SPI 100 / SI 253` to `SFF3`.

### 8.3 "Temporal" & "Pose" Control Paths

`SFF2` also implements the two reverse chains:

```
"Temporal" : Encoder plain -> SFF2 -> "SPI 200 / SI 255" -> SFF1
"Pose"     : SFF3 "SPI 300 / SI 255" -> SFF2 -> plain -> Decoder
```

The `Temporal` classifier accepts exactly the packed 16-byte payload from the `Encoder`-facing port. The `Pose` classifier accepts exactly one base service header plus the packed 24-byte command from the `SFF3`-facing port. These controls do not share primary-frame state.

### 8.4 Burst Ownership

Packets are forwarded through bounded bursts while preserving explicit ownership. Accepted packets are removed from the local burst, non-accepted packets remain eligible for local resubmission, & abandoned entries are explicitly released when the bounded retry policy expires. The final representative runs present no primary-route partial accepts or zero-accept pressure within `SFF2`.

### 8.5 Proxy State & Unaware Service Functions

The proxy model remains central to the thesis methodology. `Encoder` & `Decoder` parse no `nsh_hdr`; instead, `SFF2` retains the `SPI` / `SI` transition around them. This makes the service chain observable without contaminating application parsers with service-routing responsibilities.

The relevant primary sequence is:

```
255 -> Encoder -> 254 -> Decoder -> 253 -> SFF3
```

`SFF3` subsequently consumes the final aware state & returns the application packet to a plain end-device form.

### 8.6 Route-Specific Payload Semantics

Route 0 treats point-cloud bytes as `geo_agg_hdr + cam_hdr + point_tx[]`; Route 1 treats application bytes as `cam_hdr + enc_hdr + MPEG-TS`; Route 2 treats them as `dec_hdr + point_tx[]`. This distinction is reflected directly in the `rx_points`, `tx_points`, `rx_media_bytes`, `tx_media_bytes`, `payload_bytes`, reference-size, & integrity calculations.

All three route logs contain exactly 300 rows in both final archived conditions.

### 8.7 SFF2 Telemetry — Complete Semantics

Each route exports the same 42-column schema, allowing cross-route analysis while retaining route-specific point / media interpretations:

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_media_bytes;tx_media_bytes;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;camera_node_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

The resulting files are:

```
/shared/log/sff2/telemetry_sff1_enc.csv
/shared/log/sff2/telemetry_enc_dec.csv
/shared/log/sff2/telemetry_dec_sff3.csv
```

---

## ⚙️ 9. Encoder — Hybrid Frame Processing, "CUDA", "Temporal" Control, & "H.265"

### 9.1 Role

The `Encoder` is deliberately engineered to remain "NSH"-unaware. It processes standard "UDP" packets where the application payload initiates with the geometric context imparted by the `SFF2` proxy:

```text
[ geo_agg_hdr | cam_hdr | points ]
```

Its functional paradigm integrates two distinct execution models.

Upon packet ingress, it functions in a data-plane-oriented capacity: coordinates are decoded & written instantaneously into deterministic frame offsets, packet-completion status is incremented continuously, & the optimal geometry snapshot is preserved. Simultaneously, during intensive "CPU" / "GPU" cycles, the implementation sustains network reception through cooperative polling.

Conversely, at the projection phase, a stringent frame-level readiness constraint is enforced, given that the exact active point set & definitive geometry must be fully resolved. Consequently, the `Encoder` constitutes a **hybrid frame-aware entity**, uniquely positioned between a traditional passive application awaiting complete objects & an atomic packet-local data-plane operation.

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

Packet sequence numbers definitively govern destination offsets. Therefore, point conversion executes progressively as packets arrive, negating the necessity for an independent post-reception frame conversion traversal.

A standard frame is deemed processable exclusively when all anticipated packets & points are successfully aggregated. Upon an "EOS" condition, a fragmentary final frame may undergo compaction & processing; however, the validated 300-frame test suite comprises only complete frames.

### 9.3 Selecting the Most Complete Geometry Snapshot

`SFF1` continuously broadcasts progressive geometry. Consequently, the `Encoder` isolates the snapshot possessing the maximum valid `active_point_count` identified for that specific frame.

For a finalised frame, `geometry_from_sff1()` authorises the offloaded metrics strictly when:

```text
metadata_active_points == active_point_count
```

& all decoded geometry components demonstrate finiteness & internal consistency.

This mechanism precludes an early progressive packet from being erroneously interpreted as the definitive geometric description.

### 9.4 Frame Readiness & Incomplete Frames

The conventional processing trajectory demands absolute point completeness preceding projection, as the six-view representation & "codec" input inherently rely on frame-level aggregates.

However, this barrier does not imply that antecedent tasks are postponed. Prior to achieving readiness, the `Encoder` has proactively:

```text
validated packet headers
converted point coordinates
placed points at deterministic offsets
updated receive counters
tracked geometric metadata progression
recorded arrival timing
```

The residual barrier is strictly confined to operations yielding inherently frame-level results, avoiding passive stagnation across the computational pipeline.

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
max_r_ms                = 0
```

Should metadata prove unavailable, inconsistent, deactivated, or partial, `geometry_recompute_local()` provides a robust functional fallback. It reconstructs sums / extrema, derives the centroid / extent / box centre, & executes the radius pass across the active point set while persistently polling "DPDK".

The ultimate object scale, instituted prior to projection, hinges upon the radius target:

```text
target_radius = CAMERA_DISTANCE * 0.2
final_scale   = target_radius / max_r
```

contingent upon implementation-level validity verifications.

This selectable trajectory holds profound experimental significance: it facilitates a highly controlled `OFFLOAD_MODE_ENABLED` versus `OFFLOAD_MODE_DISABLED` comparative analysis without altering the underlying packet-processing architecture.

### 9.6 Workload-Driven "Temporal" Controller

"Temporal" adaptation remains centralised at `Encoder`. For each eligible service sample `T_n`, the controller maintains:

```text
T_base           = 1000 / TARGET_FPS
T_budget( skip ) = skip * T_base
E_n              = EWMA_ALPHA * T_n + ( 1 - EWMA_ALPHA ) * E_( n - 1 )
workload_ratio   = E_n / T_budget( active_skip )
```

Current parameters are:

```text
TARGET_FPS        = 30
EWMA_ALPHA        = 0.25
MAX_SKIP          = 9
MIN_FRAMES        = 3
STABLE_STREAK     = 3
MAX_FRAMES        = 15
OVERLOAD_STREAK   = 2
RECOVERY_STREAK   = 9
RETRY_FRAMES      = 3
OVERLOAD_RATIO    = 0.90
RECOVERY_RATIO    = 0.75
OVERLOAD_FRACTION = 0.25
RECOVERY_FRACTION = 0.10
```

After the warm-up / stability gate, overload can be asserted when any of the following holds:

```text
workload_ratio >= OVERLOAD_RATIO
raw_queue_ms   > active_budget_ms * OVERLOAD_FRACTION
frame_backlog  >= 2
"codec" backlog is observed to be growing
```

The recovery predicate is intentionally stricter: the projected ratio at `skip - 1` must not exceed `RECOVERY_RATIO`, `raw_queue_ms` must remain within the recovery fraction, `frame_backlog` must be zero, & "codec" backlog must not be increasing. Consecutive overload / recovery streaks determine when the requested factor changes.

A requested skip is not considered semantically complete merely because a control datagram was sent. If the returned `Camera` stream does not yet reflect the requested factor, the `Encoder` re-dispatches the same request after the configured three-frame interval. This preserves a bounded local retry plus application-level confirmation model without introducing transport retransmission.

Logged events remain:

```text
"WARMUP"
"IDLE"
"SKIP+1"
"SKIP-1"
"RETRY"
"INVALID"
```

Both final representative runs contain five `WARMUP` rows followed by 295 `IDLE` rows. `frame_backlog = 0` throughout; NON-QUALITY reaches a maximum `workload_ratio` of `0.386`, while QUALITY reaches `0.499`. Neither satisfies the overload condition & all 300 application frames retain `current_skip = 1`.

### 9.7 "CUDA" Memory Strategy

The projection pipeline proactively preallocates persistent device buffers, a designated "CUDA" stream, & precise timing events. Host "I420" output slots are similarly allocated at inception & formally registered with "CUDA".

Current buffering frameworks incorporate:

```text
H2D_CHUNK_POINTS = 65536
YUV_BUFFER_COUNT = 3
```

The point array is transferred asynchronously via designated chunks. Prior to each chunk transfer, the "CUDA" path asserts the capacity to invoke the `Encoder`'s "DPDK" polling callback. Following kernel initiation & subsequent copy-back, the worker continually polls while `cudaStreamQuery()` denotes the stream as incomplete.

This configuration strictly curtails the duration wherein "GPU" submission obstructs packet reception & circumvents repetitive device allocations along the measured pathway.

### 9.8 "CUDA" Projection Stages

The source-side pose carried by the `Camera` / `Encoder` path remains the neutral reference. Dynamic `User`-originated stance is intentionally applied later by `Decoder`; consequently, the `Encoder` can preserve its projection geometry independently from the reverse "Pose" service path.

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

Dynamic `User` "Pose" support is intentionally isolated downstream at `Decoder`; only a future migration of pose-dependent variables into the `Encoder` projection stage would require reassessment of this optimisation.

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

The directive `atomicMax()` mitigates visibility conflicts within the integer depth buffer. This paradigm localises intensive point-parallel computations upon the "GPU", leaving only streamlined geometric control formulations resident on the host.

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

"CUDA" event chronologies isolate strictly asynchronous "GPU" operations, whereas `projection_ms` captures the full host-visible interval enveloping the projection call. Given that cooperative "DPDK" polling proceeds while the stream maintains an incomplete status, `host_overhead_ms` signifies residual host-visible time rather than acting as a definitive arithmetic-"CPU" kernel metric.

### 9.12 Persistent "FFmpeg" / "NVENC" Process & Pre-Roll

A singular, persistent "FFmpeg" process initiates concurrently with the experiment. Contemporary rate-control & latency-oriented parameters specify:

```text
"codec"        = hevc_nvenc
preset         = p2
tune           = ull
rate control   = cbr
target bitrate = 10M
buffer size    = 20M
"GOP"          = 15
B frames       = 0
lookahead      = 0
delay          = 0
zero latency   = 1
flush packets  = 1
```

Preceding the measurement of application frames, the `Encoder` propels one "GOP" consisting of private, unrecorded pre-roll frames:

```text
FRAMES = 15
```

This pre-roll sequence traverses the standard writer & parser conduits, propelling the persistent "codec", muxer, & attribution logic into an operational, warmed state. Crucially, pre-roll frames bypass application network transmission entirely & remain omitted from the 300-row `Encoder` telemetry.

A dedicated writer thread & three designated "I420" slots uncouple the projection phase from blocking interactions with "FFmpeg" stdin. In instances where all slots are committed, the `Encoder` perpetually services network / "codec" activities while awaiting slot emancipation.

### 9.13 Why "MPEG-TS" / "PES" Parsing Is Necessary

Pipe reads sourced from "FFmpeg" inherently fail to preserve distinct video-frame boundaries. Thus, the `Encoder` is obliged to reconstruct strictly structured 188-byte "MPEG-TS" packets from variable read sizes & identify precise video "PES" demarcations.

The initial identified video-"PES" boundary corresponds directly to the oldest frame assigned to "FFmpeg". This enables the derivation of:

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

The final `Encoder` exports **69 fields**, making explicit the distinction among raw point reception, offloaded / fallback geometry, "GPU" projection, "codec" submission, asynchronous output attribution, "Temporal" control state, objective "luma" quality, & Tx acceptance:

```text
frame_id;rx_complete;tx_complete;current_skip;event;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;codec_exit_time;node_exit_timestamp;original_points;rx_points;processed_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;conversion_ms;geometry_aggregation_ms;max_r_ms;projection_ms;codec_write_ms;active_tx_ms;active_process_ms;reference_process_ms;total_processing_ms;total_residency_ms;reference_residency_ms;node_efficiency_pct;reference_efficiency_pct;gpu_transfer_ms;gpu_kernel_ms;gpu_packing_ms;gpu_copyback_ms;host_overhead_ms;camera_node_ms;e2e_latency_ms;reference_e2e_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;reference_jitter_ms;raw_queue_ms;render_queue_ms;workload_ewma_ms;workload_ratio;frame_backlog;codec_backlog;encode_service_ms;encode_h265_ms;mse_y;psnr_y;ssim_y;mpeg_bytes_generated;ffmpeg_write_calls;ffmpeg_write_eagain;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```

| Field / Group | Exact Semantics |
|---|---|
| `frame_id`, `rx_complete`, `tx_complete`, `current_skip`, `event` | Native frame identity, input / compressed-output completion, active temporal factor, & controller state. |
| `yaw`, `pitch`, `zoom` | Source-projection pose retained for compatibility. Dynamic `User` pose is applied later by `Decoder`. |
| source / node timestamps | `Camera` source anchor, first `Encoder` arrival, first attributed "codec"-output frontier, & final "DPDK" egress. |
| `original_points`, `rx_points`, `processed_points` | Declared source population, unique received points, & population projected by the `Encoder`. |
| `rx_packets`, `tx_packets`, `payload_bytes`, `reference_size_bytes` | Raw input segmentation, compressed output segmentation, point payload, & reference comparison size. |
| throughput / bitrate fields | Separate receive-span throughput, logical compressed bitrate, protocol-inclusive output bitrate, & reference baseline. |
| `conversion_ms` | Network-to-host point conversion & deterministic placement accumulated over input packets. |
| `geometry_aggregation_ms`, `max_r_ms` | Local fallback geometry work. These remain zero when validated `SFF1` offload metadata is consumed. |
| `projection_ms` | Host-visible full "CUDA" projection interval. |
| `codec_write_ms` | Writer-side interval associated with submitting the projected frame to persistent "FFmpeg". |
| `active_tx_ms` | Active compressed "DPDK" egress calls. |
| `active_process_ms`, `reference_process_ms`, `total_processing_ms` | Measured active native work, reference-process comparator, & project-defined processing aggregate. They must not be blindly added to asynchronous "codec" latency. |
| `total_residency_ms`, `reference_residency_ms` | Measured / reference frame residence boundaries. |
| `node_efficiency_pct`, `reference_efficiency_pct` | Active-work ratios against measured / reference residence. |
| `gpu_transfer_ms`, `gpu_kernel_ms`, `gpu_packing_ms`, `gpu_copyback_ms`, `host_overhead_ms` | "CUDA" event-derived sub-stages & remaining host-visible projection overhead. |
| `camera_node_ms`, `e2e_latency_ms`, `reference_e2e_ms` | `Camera`-to-`Encoder` timing frontiers. `reference_e2e_ms` ends at frame readiness & therefore differs from final compressed-output exit. |
| `schedule_delay_ms`, `instant_jitter_ms`, `desynced_jitter_ms`, `reference_jitter_ms` | Source cadence / gap-aware variability measures. |
| `raw_queue_ms` | Frame readiness to `Encoder` service start. |
| `render_queue_ms` | Projection completion to "codec"-writer submission. |
| `workload_ewma_ms`, `workload_ratio` | Smoothed service signal & fraction of the active temporal budget. |
| `frame_backlog`, `codec_backlog` | Native pending frame count & observed writer / "codec" depth. "Codec" growth participates in overload / recovery evaluation. |
| `encode_service_ms` | Service interval used for `Encoder` workload characterisation. |
| `encode_h265_ms` | Genuine-frame "codec" submission to first attributed video-"PES" output frontier. |
| `mse_y`, `psnr_y`, `ssim_y` | Post-stream "luma" fidelity components; populated only with `QUALITY_CAPTURE = 1`. |
| `mpeg_bytes_generated` | Application-attributed "MPEG-TS" bytes before network packetisation. |
| `ffmpeg_write_calls`, `ffmpeg_write_eagain` | Pipe-write call count & nonblocking defensive retry observation. |
| Tx acceptance / starvation fields | Local compressed-output zero / partial accepts, resubmission attempts, & mbuf allocation failures. |

When `OFFLOAD_MODE_ENABLED` receives a final geometry snapshot whose `active_point_count` agrees with the assembled frame:

```text
geometry_aggregation_ms = 0
max_r_ms                = 0
```

The main workload variables are interpreted as:

```text
T_base           = 1000 / TARGET_FPS
T_budget( skip ) = skip * T_base
E_n              = EWMA_ALPHA * service_n + ( 1 - EWMA_ALPHA ) * E_( n - 1 )
workload_ratio   = E_n / T_budget( skip )
```

`encode_h265_ms` remains an asynchronous "codec"-output latency & must not be arithmetically merged with `active_process_ms` or `total_processing_ms` as though the intervals were disjoint serial stages.

### 9.16 Encoder "End-of-Stream" Handling

Upon detecting "EOS", the `Encoder` initiates the finalisation of any eligible residual frame, drains lingering writer commitments & encoded outputs, seals the "FFmpeg" input channel, completes the parser exhaustion process, flushes residual "DPDK" output, & solely thereafter records the terminal telemetry data.

This sequential ordering remains essential because encoded bytes frequently linger within the "codec" / muxer pipeline long after the concluding application frame submission. Consequently, the terminal frame can routinely exhibit elevated residency or compressed-media delays in contrast to steady-state frames; thus, it demands analysis as an authentic tail-drain circumstance rather than suffering unceremonious deletion.

---

### 9.17 Optional Post-Stream "Luma" Quality Assessment

`Encoder` additionally exposes an explicit `QUALITY_CAPTURE` mode whose objective is to quantify coding distortion without introducing persistent disk activity into the measured streaming interval. When enabled, the node reserves bounded in-memory regions for both the authentic projected `Y` planes & the byte-preserved application-attributed "MPEG-TS" stream. Each genuine application frame contributes its "luma" reference through `write_reference()`, whilst encoded chunks are appended through `write_stream()`.

The streaming hot path therefore performs bounded memory copies exclusively; serialization to `reference_y.raw` & `encoded.ts`, alongside the subsequent objective analysis, occurs only after "EOS". Two isolated "FFmpeg" filter runs recover frame-associated `MSE-Y`, `PSNR-Y`, & `SSIM-Y`, whose results are merged back into `telemetry_encoder.csv`. Temporary analysis files are deleted only when every captured application frame has received complete indicators.

This mechanism deliberately distinguishes **real-time execution** from **offline quality evaluation**. The quality run must not be interpreted as a browser-interaction benchmark, while the interactive run deliberately leaves these columns unset in order to avoid contaminating the latency experiment with the additional fidelity capture.

## 🎞️ 10. Decoder — Persistent Hardware Decode, Reconstruction, & Dynamic "Pose"

### 10.1 Role

`Decoder` constitutes the second "SFC"-unaware application function. It receives standard "UDP" datagrams from the `SFF2` proxy, consumes the attributed "MPEG-TS" stream, reconstructs the projected point cloud, applies the latest `User` "Pose", & emits plain `dec_hdr + point_tx[]` packets back to `SFF2`.

### 10.2 Persistent "FFmpeg" / Hardware Decode

A persistent "FFmpeg" child is established using "CUDA" hardware acceleration & `hevc_cuvid`. Encoded "MPEG-TS" arrives through a bounded packet queue, while a dedicated writer thread feeds the child pipe. Both the writer & the child are explicitly pinned to logical Core `2`, avoiding scheduler migration into the "DPDK" / reconstruction core.

The internal "codec" queue & decoded-frame staging parameters are:

```
QUEUE_SIZE        = 16384
WRITE_BATCH_SIZE  = 65536
I420_BUFFER_COUNT = 3
FFMPEG_CPU        = 2
```

`Decoder` preserves application frame attribution separately from the raw "FFmpeg" stream chronology, as private warm-up / drain material prevents `ffmpeg.txt` rows from being interpreted as a one-to-one native frame index.

### 10.3 Pre-Roll & Readiness Coordination

`Decoder` startup is part of the deterministic launcher sequence. The decoder prepares its persistent "codec" state & exposes `/tmp/sfc-decoder-ready`; the `Encoder` waits for this readiness condition before the source path is ultimately admitted. This prevents initial stream traffic from racing an uninitialised decoding pipeline.

### 10.4 Reconstruction Pipeline

Each decoded `I420` image encodes the vertically stacked six-face `Geometry` / `Texture` / `Occupancy` representation emitted by `Encoder`. Reconstruction executes through a fixed "CUDA" path:

```
Host decoded "I420"
  -> "H2D" transfer
  -> 2 x 2 occupancy erosion
  -> per-face 3D reconstruction
  -> dynamic "Pose" kernel
  -> compact valid-point result
  -> "D2H" transfer
```

The native node records `arrived_points`, `eroded_points`, & `valid_points` independently, making representation / occupancy reduction distinct from network integrity. Specifically, `arrived_points` measures the loose-threshold occupancy population preceding erosion, `eroded_points` records the corresponding loose-threshold population remaining after erosion, & `valid_points` identifies the stricter reconstruction-threshold population actually admitted to the final point cloud. Thus, `eroded_points` denotes the post-erosion population rather than the number of points removed by erosion.

### 10.5 Dynamic "Pose" Application

`Decoder` receives a separate plain 24-byte "Pose" command from `SFF2`. A command is accepted only when its timestamp is newer than the currently retained control state. Reconstruction takes a stable snapshot of yaw, pitch, & zoom, applies the transformation in the "GPU" pipeline, & records `pose_control_ms` from the originating command timestamp to the measured application frontier.

"Pose" does not force `Encoder` reprojection. This preserves the architectural separation between the compression representation & end-user spatial interaction.

### 10.6 Cooperative Network Servicing

The main "DPDK" / reconstruction role remains bound to Core `4`, while "FFmpeg" I/O resides on Core `2`. Cooperative network callbacks continue processing datagrams around "codec" / "GPU" waits, limiting the opportunity for point or control traffic to accumulate merely because a frame is undergoing reconstruction.

### 10.7 Downstream Packetisation

Completed reconstruction is segmented at `POINTS_PER_PACKET = 80`. Every packet carries the 52-byte `dec_hdr`; the last packet is identifiable through exact point / sequence counts rather than a transport-level stream boundary. Empty valid frames retain a header-only representation with zero point population.

### 10.8 End-of-Stream Handling

"EOS" causes `Decoder` to close "codec" input, drain queued compressed material, consume remaining decoded outputs, issue the post-roll completion marker expected by `Encoder`, transmit a downstream terminal packet, & persist telemetry only after the attributed application state has been resolved.

### 10.9 Decoder Telemetry

The final 61-column schema is:

```text
frame_id;rx_complete;tx_complete;current_skip;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_media_bytes;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbps;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;arrived_points;eroded_points;valid_points;erosion_ms;reconstruction_ms;pose_ms;reconstruction_pipeline_ms;tx_duration_ms;active_tx_ms;active_process_ms;reference_process_ms;total_processing_ms;total_residency_ms;reference_residency_ms;node_efficiency_pct;reference_efficiency_pct;gpu_transfer_ms;gpu_copyback_ms;host_overhead_ms;camera_node_ms;e2e_latency_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;pose_control_ms;codec_queue_ms;frame_queue_ms;codec_backlog;decode_service_ms;decode_h265_ms;ffmpeg_write_calls;ffmpeg_write_failures;codec_queue_drops;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```
The `Decoder`-specific indicators isolate compressed-media ingestion, hardware-decoding behaviour, reconstructed-geometry processing, dynamic pose application, internal queueing, & output generation:

| Field / Group | Exact Semantics |
|---|---|
| `rx_media_bytes` | Application-attributed compressed "MPEG-TS" bytes received for the corresponding frame prior to hardware decoding. |
| `arrived_points` | Loose-threshold occupancy population recovered from the decoded representation prior to morphological erosion. |
| `eroded_points` | Loose-threshold occupancy population remaining after erosion. Despite the field name, this value records the post-erosion population, not the number of points removed. |
| `valid_points` | Final point population admitted by the stricter reconstruction threshold after erosion & emitted toward the downstream path. |
| `erosion_ms` | "GPU" interval consumed by the occupancy-erosion operation preceding final point reconstruction. |
| `reconstruction_ms` | "GPU" interval dedicated specifically to converting the valid projected representation back into three-dimensional point coordinates. |
| `pose_ms` | "GPU" interval required to apply the active `yaw`, `pitch`, & `zoom` transformation to the reconstructed geometry. |
| `reconstruction_pipeline_ms` | Complete host-observed reconstruction interval spanning device preparation, erosion, point reconstruction, "Pose" application, copy-back, & associated orchestration. |
| `gpu_transfer_ms` | Time attributed to the host-to-device transfer / preparation operations required before reconstruction kernels can execute. |
| `gpu_copyback_ms` | Time required to recover the completed reconstructed point population from device-visible memory to the host-side output representation. |
| `host_overhead_ms` | Portion of `reconstruction_pipeline_ms` not directly accounted for by the individually measured "GPU" transfer, erosion, reconstruction, pose, & copy-back intervals. |
| `pose_control_ms` | Delay from the timestamp of a newly accepted `User` "Pose" directive to completion of its corresponding transformation inside the reconstruction pipeline. It is populated only when an actual pending pose generation is measured. |
| `codec_queue_ms` | Delay from the first arrival of the attributed compressed frame at `Decoder` until the associated media begins submission toward the persistent "FFmpeg" decoder. |
| `frame_queue_ms` | Delay between availability of a decoded frame & the instant at which the native reconstruction service begins processing that decoded output. |
| `codec_backlog` | Maximum compressed-packet queue occupancy observed while receiving the application frame, exposing pressure between "DPDK" ingestion & the dedicated "FFmpeg" writer. |
| `decode_service_ms` | Host-side decoded-output service span beginning when `Decoder` starts retrieving a genuine post-pre-roll "I420" frame from persistent "FFmpeg" output & ending when the complete decoded frame buffer becomes available. |
| `decode_h265_ms` | Broader "H.265" latency measured from the first attributed compressed-data submission toward "FFmpeg" until the corresponding decoded frame becomes available. It therefore includes "codec"-side waiting in addition to the immediate decode service interval. |
| `ffmpeg_write_calls` | Number of writer-side pipe submission operations attributed to the frame while feeding the persistent "FFmpeg" process. |
| `ffmpeg_write_failures` | Number of unsuccessful persistent-decoder pipe write operations attributed to the frame. |
| `codec_queue_drops` | Number of compressed packet jobs attributed to the frame that could not be admitted to the bounded decoder "codec" queue. |
| `reference_process_ms` | `Decoder` reference-work comparator formed from `decode_service_ms` plus the reconstruction pipeline with `pose_ms` removed, thereby exposing the processing cost independently from active `User`-driven "Pose" transformation. |
| `reference_residency_ms` | Reference residency terminating at reconstruction completion rather than final downstream packet transmission. |
| `reference_efficiency_pct` | Ratio between `reference_process_ms` & `reference_residency_ms`, retained as a comparison frontier distinct from the complete node-efficiency definition. |
| `total_processing_ms` | Full frame-associated `Decoder` interval extending from the first attributed "codec" submission to the final node-exit frontier. Unlike `active_process_ms`, it intentionally includes intervening asynchronous waiting periods. |

---

## 🧭 11. SFF3 — Final Aware Boundary & Reverse "Pose" Classifier

### 11.1 Primary-Path Role

`SFF3` receives the reconstructed primary stream exclusively from `SFF2`. It expects the base service state:

```
MAIN_SPI        = 100
MAIN_SI_DECODER = 253
```

The node validates "Ethernet" / "IPv4" / "UDP" / service fields, enforces frame sequence & point-count consistency, removes the 8-byte service envelope, rewrites the local `SFF3`-to-`User` network header, & forwards the unchanged `dec_hdr + point_tx[]` application body.

### 11.2 Reverse "Pose" Classification

The `User`-facing port also accepts a plain 24-byte `pose_payload`. `SFF3` validates the exact packet size & command fields, then constructs the reverse aware envelope:

```
POSE_SPI = 300
POSE_SI  = 255
```

The original application command is preserved byte-for-byte behind the service base so that `SFF2` can decapsulate it for `Decoder` without translating pose semantics.

### 11.3 Backpressure & Integrity

`SFF3` uses the same 4096-descriptor queues, 32-packet burst size, & bounded zero-accept policy as the remaining native chain. Its telemetry explicitly distinguishes primary Rx completion from Tx completion, which proved essential for diagnosing receiver-side backpressure during development. In both final archived runs, all 300 primary frames are complete in both directions with zero protocol errors, zero partial accepts, & zero Tx zero-accept events.

### 11.4 Telemetry

`telemetry_sff3.csv` contains 40 fields:

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;camera_node_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

---

## 🖱️ 12. User — Final Reassembly, Interaction, Web Publication, & Quality Capture

### 12.1 Native End-Device Role

The native `User` process is the terminal "DPDK" consumer & the origin of the reverse "Pose" command. It validates `SFF3`-provided plain "UDP" packets, reassembles exact `dec_hdr` sequences, converts point coordinates to host representation, gathers end-to-end telemetry, & exposes complete snapshots through shared memory rather than binding the "DPDK" loop directly to browser networking.

### 12.2 Shared-Memory Publication

Two shared objects are used:

```
/dev/shm/frame.bin
/dev/shm/ctrl.bin
```

`frame.bin` contains the 72-byte `web_hdr` followed by the current `host_point[]` payload. An odd / even sequence marker prevents the bridge from copying a frame while the native process is still mutating it. `ctrl.bin` provides the reverse command / acknowledgment channel.

In `QUALITY_CAPTURE = 1`, frame publication & sequence updates are intentionally disabled because the "Web" path is absent. The same native reassembly logic remains active, preserving a common application contract across validation modes.

### 12.3 Fire-&-Forget "Pose" Dispatch & Command Tracking

A browser "Pose" query receives a monotonically increasing command identifier. The native process records the dispatch timestamp, requested yaw / pitch / zoom, first returning reference observation, first matching applied frame, & optional browser photon acknowledgment. Dispatch itself is non-blocking from the user-interaction perspective.

If an active command has not yet become observable in the returned stream, `User` re-presents it every:

```
RETRY_FRAMES = 3
```

This application-level re-presentation preserves the "UDP" transport model while protecting command observability from a transient lost control datagram.

### 12.4 Asynchronous "Python" / "WebSocket" Bridge

The bridge executes independently from the "DPDK" loop. Each connected peer owns:

```
asyncio.Queue( maxsize = 1 )
one frame_ready acknowledgment gate
```

`frame_loop()` observes only the shared sequence identifier & enqueues a lightweight availability token. It does **not** copy point payloads pre-emptively. `send_loop()` waits until the browser has acknowledged the preceding rendered frame, then copies whichever completed shared snapshot is latest at that moment. Consequently, stale unsent geometries are discarded naturally rather than accumulating as a FIFO backlog.

The "WebSocket" server runs with:

```
compression = None
```

while the "HTTP" server occupies an independent thread. In the final interactive launcher, this "Python" bridge runs on Core `2` with `nice -n 5`, deliberately giving the shared `Decoder` "codec" / writer work superior scheduler priority on that logical "CPU".

### 12.5 Three.js Viewer

The viewer employs `Three.js r128` with `OrbitControls`. Geometry storage expands in blocks of `65536` points, while the "GPU"-facing representation uses compact `Float32` XYZ & normalised `Uint8` RGB attributes. Rendering is event-driven through `requestAnimationFrame()`; there is no unconditional continuous animation loop.

Controls are:

```
"W" / "S" -> pitch
"A" / "D" -> yaw
"+" / "-" -> zoom
"R"       -> reset
```

Each angular step is `pi / 36` ( 5 degrees ) & each zoom modification uses a `1.05` multiplicative factor. A `streamReady` gate prevents a pose from being sent before the first genuine reconstructed frame establishes the active baseline; a locally accumulated pending "Pose" is dispatched immediately afterward.

### 12.6 Browser Acknowledgment & "Command-to-Photon"

The browser records `performance.now()` when a command is issued. When a subsequently received frame carries the matching command identifier, that timestamp remains associated until the frame has actually been rendered. Only after `renderer.render()` does the viewer calculate `Command-to-Photon`, delete the command entry, & send a JSON acknowledgment containing the rendered frame, command, & "CTP" value.

This acknowledgment serves two purposes: telemetry & release of the one-frame-in-flight Web gate. It does not convert "Pose" dispatch into a blocking request / response interface.

### 12.7 Quality Capture & `Gauge` Synchronization

When `QUALITY_CAPTURE = 1`, no "Python" bridge, "HTTP" server, "WebSocket" server, or browser process is launched. A 1-GiB in-memory `quality_buffer` receives complete reconstructed frame records by copying the stable `web_points` content after native reassembly. Persistent serialization to `results.bin` occurs only after "EOS".

The terminal protocol is:

```
User closes / serializes quality capture
  -> User writes telemetry_user.csv
  -> User creates /tmp/sfc-user-quality
  -> Gauge starts offline analysis
  -> Gauge merges geometry metrics into telemetry_user.csv
  -> Gauge creates /tmp/sfc-user-done
  -> User prints final "End of stream detected..."
```

`Gauge` runs serially. It first reverses the applied pose, then applies statistical filtering & robust "ICP" alignment before calculating symmetric nearest-neighbour metrics. Its fixed parameters include:

```
VOXEL_MM        = 1.820
ICP_RADIUS      = 200.0 / VOXEL_MM
ICP_REPETITIONS = 30
ICP_POINTS      = 60000
OUT_K           = 20
OUT_STD         = 2.0
```

All `cKDTree` operations explicitly employ one worker. The serial design is a post-stream methodological choice favouring determinism & simplicity; it does not execute concurrently with the measured 300-frame "DPDK" stream.

`Gauge` defines the expected assessment population from `User` rows satisfying `rx_complete = 1`. A frame contributes a completed objective result only when its capture record & reference geometry are both available & the metric pipeline returns successfully. The temporary `results.bin` capture is removed exclusively when the completed population equals the expected one; the final archived quality run therefore evaluates all 300 eligible frames rather than silently tolerating missing geometric assessments.

### 12.8 User Telemetry

The final 47-column schema is:

```text
frame_id;rx_complete;current_skip;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;arrived_points;eroded_points;valid_points;rx_points;rx_packets;payload_bytes;data_integrity_pct;internal_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;arrival_pct;erosion_pct;valid_pct;web_publish_ms;web_ack_ms;active_process_ms;total_residency_ms;node_efficiency_pct;camera_node_ms;e2e_latency_ms;reference_e2e_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;cmd_id;reference_cmd_ms;cmd_apply_ms;cmd_photon_ms;quality_save_ms;mean_error;geom_rmse;chamfer;hausdorff;mean_mm;rmse_mm;chamfer_mm;hausdorff_mm
```

The final eight geometry columns remain unset in the interactive run & are populated by `Gauge` only in quality mode.

Moreover, `User`-specific indicators describe the terminal reconstruction population, browser-publication frontier, interactive "Pose" chronology, & optional post-stream objective-quality evaluation:

| Field / Group | Exact Semantics |
|---|---|
| `arrival_pct` | Percentage of the original source-frame population represented by the reconstruction candidates reported by `Decoder`, computed as `arrived_points / original_points * 100`. |
| `erosion_pct` | Percentage of loose-threshold reconstruction candidates surviving the `Decoder` erosion stage, computed as `eroded_points / arrived_points * 100`. |
| `valid_pct` | Final valid reconstructed population expressed against the original source cloud, computed as `valid_points / original_points * 100`. |
| `web_publish_ms` | Native `User` time required to finalise the stable shared-memory Web header & publish the completed frame through the odd / even sequence protocol. It remains zero during `QUALITY_CAPTURE = 1`, where Web publication is disabled. |
| `web_ack_ms` | Delay from native publication of a frame to reception by `User` of the corresponding browser render acknowledgment through the asynchronous control mapping. |
| `reference_e2e_ms` | `Camera`-to-`User` latency terminating when the complete reconstructed frame becomes available natively, before optional Web publication extends the final node-exit frontier. |
| `cmd_id` | Identifier of the "Pose" directive whose requested state is first observed on the corresponding returned reconstructed frame. Zero identifies frames not associated with a newly matched directive. |
| `reference_cmd_ms` | Delay from native dispatch of a "Pose" directive to arrival of the first subsequently observed complete frame used as the command-reference frontier, independently from whether its pose already matches the request. |
| `cmd_apply_ms` | Delay from directive dispatch until the first complete returned frame whose `yaw`, `pitch`, & `zoom` values actually match the requested pose. |
| `cmd_photon_ms` | Browser-measured "Command-to-Photon" latency: interval from command generation in the viewer to rendering of the first frame carrying the matching directive. The value is returned asynchronously to `User` through the Web acknowledgment mapping. |
| `quality_save_ms` | In `QUALITY_CAPTURE = 1`, time required to append the completed reconstructed point-cloud snapshot to the bounded in-memory quality capture buffer. No corresponding operation is performed in the interactive condition. |
| `mean_error` | Directed reconstructed-to-reference mean nearest-neighbour geometric error obtained after inverse-pose normalisation, statistical filtering, & robust "ICP" registration against the original reference frame. Populated by `Gauge` only after a quality run. |
| `geom_rmse` | Symmetric root-mean-square geometric deviation derived from reconstructed-to-reference & reference-to-reconstructed nearest-neighbour distances after registration. |
| `chamfer` | Symmetric Chamfer distance obtained as the sum of the two directional mean nearest-neighbour distances. |
| `hausdorff` | Symmetric Hausdorff distance, representing the maximum nearest-neighbour deviation observed in either geometric direction. |
| `mean_mm` | `mean_error` converted from dataset voxel coordinates to millimetres through `VOXEL_MM = 1.820`. |
| `rmse_mm` | `geom_rmse` expressed in millimetres. |
| `chamfer_mm` | Symmetric Chamfer distance expressed in millimetres. |
| `hausdorff_mm` | Symmetric Hausdorff distance expressed in millimetres. |

---

## 🖥️ 13. "CPU", "GPU", & Core-Constrained Execution

The host exposes four physical cores / eight logical "CPU" threads with sibling pairs:

```text
{ 0, 4 }
{ 1, 5 }
{ 2, 6 }
{ 3, 7 }
```

The final placement intentionally reuses sibling capacity across dissimilar roles while avoiding an additional core-consuming virtual switch:

| Logical Core | Final Role | Experimental Rationale |
|---:|---|---|
| `0` | `User` "DPDK"; in quality, serial `Gauge` **after "EOS"** | Terminal receive path remains isolated from the browser; offline `Gauge` reuses the same core only after streaming has ended |
| `1` | `Camera` | Source scheduling & packetisation; this logical core is now available because the former "OVS"-"DPDK" "PMD" was removed |
| `2` | `Decoder` "FFmpeg" child + writer; interactive `User` "Python" bridge at `nice +5` | "Codec" I/O is pinned; lower-priority bridge sharing is admitted only in NON-QUALITY |
| `3` | `SFF1` | Dedicated in-path geometry service function |
| `4` | `Decoder` "DPDK" / reconstruction role | Separates native packet / reconstruction work from "codec" pipe I/O |
| `5` | `Encoder` "DPDK" | Dedicated `Encoder` native processing core |
| `6` | `SFF2` | Dedicated four-port service proxy |
| `7` | `SFF3` + `Encoder` "FFmpeg" / writer | Final service boundary shares a sibling with asynchronous "codec" I/O; the measured `SFF3` load remains comparatively small |

Container cpusets are:

```text
Camera  : "1"
SFF1    : "3"
SFF2    : "6"
Encoder : "5,7" ( "DPDK" = 5, Encoder "codec" I/O = 7 )
Decoder : "2,4" ( "DPDK" = 4, Decoder "codec" I/O = 2 )
SFF3    : "7"
User    : "0,2" when QUALITY_CAPTURE = 0
User    : "0"   when QUALITY_CAPTURE = 1
```

The elimination of "OVS"-"DPDK" therefore removes the old dedicated "PMD" / auxiliary forwarding allocation from the methodology. The released scheduling capacity is consumed by the application chain itself, notably permitting `Camera` to occupy logical Core `1` directly. Since the host exposes eight logical "CPU" threads, eliminating one switch-only logical-core reservation makes `12.5 %` of the host's logical scheduling capacity available for application placement. This value describes **allocation opportunity**, not a measured 12.5 % reduction in aggregate cycles or power.

The resulting graph also exposes seven native placement units (`Camera`, `SFF1`, `SFF2`, `Encoder`, `Decoder`, `SFF3`, `User`) rather than the four coarse services of the reference architecture. Numerically this is a `75 %` increase in the number of graph-level placement units (`( 7 - 4 ) / 4 = 0.75`), but it should be interpreted as **orchestration granularity**, not as improved resource efficiency by itself. Each function has a dedicated telemetry / affinity profile, allowing a future orchestrator to move, replicate, or allocate resources to the responsible stage rather than relocating an entire monolithic `Encoder` or Client block.

No percentage claim concerning total "CPU" utilisation **against the reference implementation** is made from the supplied data. The two projects do not expose an identical per-core utilisation experiment, & the current deployment deliberately touches all eight logical threads through native, "codec", bridge, or housekeeping roles. A fair resource-efficiency comparison therefore requires a controlled co-located campaign reporting actual "CPU" time / utilisation, "GPU" occupancy, memory footprint, & energy under identical traffic. The current contribution is that such attribution is now possible per service function.

### Core `0` & Housekeeping

Core `0` remains the only non-isolated logical "CPU" when `isolcpus=1-7` is active. Assigning `User` "DPDK" to this core is an explicit experimental choice & must be retained in reproducibility records. quality analysis is not concurrent with `User` streaming: `Gauge` begins only after `/tmp/sfc-user-quality` is created following "EOS".

### Why the Current Constraint Is Manageable

The final implementation mitigates pressure through native affinity, "GPU" projection / reconstruction, persistent "codec" processes, triple image buffering, cooperative network polling, bounded descriptor queues, latest-frame-only browser publication, & source-level temporal regulation. The host remains deliberately constrained; thus, core affinity is an integral experimental variable rather than an incidental deployment detail.

---

## 14. "HugePages" & Optional "CPU" Isolation

The current `init_all.sh` resets filesystem caches & configures:

```text
nr_hugepages  = 1024
HugePage size = 2 MiB
total         = 2048 MiB ~= 2 GiB
```

The cache reset precedes topology construction. `Camera` subsequently establishes its own `WARM_MODE_ENABLED` condition, so the host reset & source warm-mode configuration remain separate reproducibility variables.

The optional isolation helper installs:

```text
isolcpus=1-7
```

leaving Core `0` as the sole non-isolated housekeeping "CPU". Enabling isolation requires a reboot:

```bash
sudo ./enable_isolcpus.sh
sudo reboot
```

After reboot:

```bash
cat /proc/cmdline
```

must confirm the expected parameter. Default scheduling is restored through:

```bash
sudo ./disable_isolcpus.sh
sudo reboot
```

`init_all.sh` & `stop_all.sh` intentionally do not alter this reboot-tier state. "HugePages", `isolcpus`, "Docker" cpusets, native "DPDK" lcores, & "codec" affinity must therefore be reported independently for every benchmark.

---

## 15. Container & Direct "DPDK" "SFC" Environment

The common image is based upon:

```text
nvidia/cuda:12.2.0-devel-ubuntu22.04
"DPDK" 22.11.4 LTS
Ubuntu 22.04
```

The build installs the native compiler toolchain, "Meson" / "Ninja", "NUMA" dependencies, "FFmpeg", "Python", & the libraries required by the browser / quality helpers. The "Python" runtime includes compatible `websockets`, `numpy`, & `scipy` packages.

Native service containers ordinarily execute with:

```text
--privileged
--net none
```

The sole exception is the interactive `User` container, which receives the "Docker" bridge plus ports `8080` / `9999` so that "HTTP" / "WebSocket" traffic can reach the local browser bridge. In quality mode those ports & the bridge are omitted.

Each service mounts:

```text
/dev/hugepages
/tmp
/shared
/app
```

### Build-Time Specialisation

The project treats compiler configuration, `"CUDA"` architecture, `"GPU"` model / driver, `"FFmpeg"` / `"NVENC"` stack, & `"DPDK"` version as constituent elements of the experimental condition. The final native compilation profile includes:

```text
"C++"  : -O3 -march=native -ffast-math -funroll-loops -std=c++14
"CUDA" : -O3 -arch=sm_61 -std=c++14
```

The `-arch=sm_61` specification fixes the generated `"CUDA"` device-code target to Compute Capability `6.1`, rather than defining an architecture-neutral numerical implementation. Consequently, cross-platform bit-level equivalence cannot be presumed when a comparison changes the `"GPU"` architecture, `"CUDA"` compiler / toolkit, or execution-driver environment. The same reproducibility constraint applies independently to the hardware `"H.265"` path, whose `"NVENC"` / `"NVDEC"` execution additionally depends upon the available `"GPU"`, NVIDIA driver, `"FFmpeg"` build, & hardware codec generation.

These factors do not invalidate cross-implementation performance or fidelity comparisons, but they prevent small numerical or codec-output differences from being attributed exclusively to the `"SFC"` / transport decomposition unless the complete hardware & software execution environment is held constant.

### Direct "SFC" Topology

`setup_topology.sh` now performs a deliberately minimal host operation: it removes stale `/tmp/sfc-*` / `/tmp/vh-*` endpoints & declares the six expected direct adjacencies. No "OVS" bridge, OpenFlow rule, "PMD" core, mirror port, or virtual-switch socket-memory policy exists in the final launch path.

`start_microservices.sh` then starts `SFF2` first, waits for its four service-side sockets, starts `Decoder` & `Encoder`, waits for their readiness markers, attaches `SFF3`, then `User`, `SFF1`, & finally `Camera`. This order converts Unix socket availability into an explicit startup dependency rather than relying upon arbitrary sleep intervals.

In NON-QUALITY the launcher pauses before `Camera` until the remote viewer has been associated. In quality mode this interactive gate is skipped because the browser path is intentionally absent.

---

## 16. Repository Structure

The completed repository organisation is conceptually:

```text
Thesis/
├── README.md
├── docs/                              # Thesis / reference material
├── env/                               # Root-level "Python" environment for offline preparation
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
    │   │   └── entrypoint.sh
    │   ├── decoder/
    │   │   ├── cpp/decoder.cpp
    │   │   ├── cu/decoder.cu
    │   │   ├── h/decoder.h
    │   │   └── entrypoint.sh
    │   ├── sff3/
    │   │   ├── c/sff3.c
    │   │   └── entrypoint.sh
    │   └── user/
    │       ├── c/user.c
    │       ├── py/user.py
    │       ├── py/gauge/gauge.py
    │       ├── html/index.html
    │       └── entrypoint.sh
    │
    ├── shared/
    │   ├── data/loot/
    │   │   ├── original/              # Original PLY sequence
    │   │   ├── bin/                   # Header-less 16-B/point reference frames
    │   │   └── made/                  # Temporary quality-capture outputs
    │   ├── log/
    │   │   ├── converter/
    │   │   ├── camera/
    │   │   ├── sff1/
    │   │   ├── sff2/
    │   │   ├── encoder/
    │   │   ├── decoder/
    │   │   ├── sff3/
    │   │   └── user/
    │   └── py/
    │       ├── converter/converter.py
    │       └── gauge/gauge.py         # Equivalent deployment location may be shared by entrypoint
    │
    ├── enable_isolcpus.sh
    ├── disable_isolcpus.sh
    ├── init_all.sh
    └── stop_all.sh
```

Exact deployment paths should follow the repository snapshot being executed; the tree above documents the logical ownership reflected by the supplied source & launcher files.

---

## 🧬 17. Dataset, "Python" Environment, & Offline Preparation

### 17.1 Research Dataset — 8i Voxelized Full Bodies ( "Loot" )

The designated experimental point-cloud resource relies upon the **"Loot" sequence traversing the 8i Voxelized Full Bodies ( 8iVFB v2 ) dataset**, graciously provided by 8i Labs & exhaustively catalogued through the JPEG Pleno database.

The foundational dataset establishes four diverse dynamic full-body sequences:

```text
longdress
loot
redandblack
soldier
```

Each sequence presents a comprehensive human subject meticulously captured via 42 "RGB" cameras systematically configured within 14 clusters, capturing at 30 frames/s for an approximate duration of 10 s. The depth-10 structure necessitates a `1024 x 1024 x 1024` voxel grid, wherein "RGB" colour attributes are rigorously assigned to occupied voxels.

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

The entire dataset remains publicly accessible via the JPEG Pleno database compliant with the attached 8i license protocols. The formally required academic citation demands:

> E. d'Eon, B. Harrison, T. Myers, & P. A. Chou, *8i Voxelized Full Bodies — A Voxelized Point Cloud Dataset*, ISO/IEC JTC1/SC29 Joint WG11/WG1 input document WG11M40059/WG1M74006, Geneva, January 2017.

Repository stakeholders must imperatively consult the primary dataset portal & corresponding license prior to any utilisation or subsequent redistribution:

```text
[https://plenodb.jpeg.org/pc/8ilabs/](https://plenodb.jpeg.org/pc/8ilabs/)
```

### 17.2 Repository Data Policy

Conspicuously, neither the authentic `.ply` frames nor the procedurally generated `.bin` frames hold presence within this repository's commit history.

This outcome is strictly intentional: the exhaustive "Loot" PLY series scales to approximately `5.14 GB` locally, while the compact binary derivative persists at roughly `3.81 GB`. Excluding both manifestations from Git rigorously guarantees a streamlined repository architecture & actively prevents fundamental source-control mechanisms from succumbing to immense experimental data weights.

Consequently, the repository securely houses the **code, data schematics, procedural conversion paradigms, & overarching telemetry**, operating on the presumption that voluminous dataset artefacts will be either independently procured or locally generated.

This capacity-driven repository policy strictly operates independent of the official dataset licence. Any local replication or expansive redistribution concerning 8i assets remains unequivocally bound by the explicit licence appended to the original dataset.

### 17.3 Binary Representation Used by the Camera

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

This robust transformation effectively isolates PLY interpretive parsing & distinct per-field numeric conversions from the `Camera`'s high-frequency streaming conduit. It definitively serves as a **storage / parsing preparatory sequence**, unequivocally void of aspirations mimicking compression algorithms grounded in rigorous information theory or explicit rate-distortion frameworks.

The prevalent scale factor anchored within the repository mandates:

```text
SCALE_FACTOR = 1.0
```

### 17.4 "Python" Environment

The definitive root-level directory:

```text
env/
```

contains the precise "Python" virtual configuration requisite for the current offline utilities, most notably the point-cloud translation apparatus.

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

Libraries encompassing `pandas` & `matplotlib` definitively lack prerequisite status regarding the execution of the primary documented converter. Although exceptionally competent regarding elevated telemetry analysis, they decisively remain extraneous to the native "DPDK" operational pathway.

### 17.5 Offline Converter

The standard execution paradigm manifests as:

```bash
source env/bin/activate
python3 src/shared/py/converter/converter.py
deactivate
```

The converter operates exclusively as an **offline preparation stage**. The resultant elapsed chronology, encompassing both PLY ingestion & BIN extrusion, must definitively eschew amalgamation with `Camera`, `SFF`, `Encoder`, "CUDA", or explicit "codec" latency quantifications.

Nonetheless, the converter telemetry presents substantial utility for replicability parameters, flawlessly tracking the strict frame population & precisely contrasting the source against the generated data footprint prevalent throughout the experiment. The definitive source-configured schema embodies:

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

`conversion_ms` is purposefully broader in scope compared to the strict calculation of `read_ascii_ms + write_bin_ms`. The deviation successfully accommodates point-array segregation, active `SCALE_FACTOR` implementation, intrinsic numeric casting, meticulous padding interpolation, & the comprehensive mapping of the 16-byte architectured representation. Consequently, it must **never** be interpreted merely as the rudimentary summation of the two distinct I / O probes.

The robust streaming outcomes explored within Section 21 inaugurate exclusively from the synthesised binary array. Ergo, Offline Converter chronologies are systematically expunged from all `Camera` / `SFF1` / `SFF2` / `Encoder` / `Decoder` / `SFF3` / `User` latency metrics & hold relevance strictly parallel to the correlative Converter telemetry tied to the dataset-preparation epoch.

---

## 🚀 18. Running the Experiment

### 18.1 Obtain & Prepare the Dataset

Acquire the 8iVFB v2 "Loot" sequence according to its original licence, place frames `loot_vox10_1000.ply` through `loot_vox10_1299.ply` beneath the expected `shared/data/loot/original` directory, & run the offline Converter before any streaming benchmark.

### 18.2 Optional: Enable "CPU" Isolation

From `src/`:

```bash
sudo ./enable_isolcpus.sh
sudo reboot
```

After reboot, verify `/proc/cmdline` before collecting results.

### 18.3 Select the Validation Condition

`start_microservices.sh` exposes the top-level experiment switch:

```bash
QUALITY_CAPTURE="0"
```

Use `0` for the interactive runtime benchmark & `1` for the isolated objective-quality run. This value is intentionally explicit rather than silently inferred.

**NON-QUALITY (`QUALITY_CAPTURE = 0`)**

```text
User cpuset          = "0,2"
User "DPDK"          = Core 0
"Python" bridge      = Core 2, nice +5
"HTTP" / "WebSocket" = enabled
viewer               = enabled
launch waits for viewer before Camera
Encoder PSNR / SSIM capture = disabled
User Gauge                  = disabled
```

**QUALITY (`QUALITY_CAPTURE = 1`)**

```text
User cpuset          = "0"
User "DPDK"          = Core 0
"Python" bridge      = disabled
"HTTP" / "WebSocket" = disabled
viewer               = absent
Camera starts after native readiness, without "ENTER" gate
Encoder "luma" quality capture = enabled
User geometry capture          = enabled
Gauge                          = serial, Core 0, strictly post-"EOS"
```

### 18.4 Start the Environment

From `src/`:

```bash
sudo ./init_all.sh
```

This resets caches, allocates 1024 2-MiB "HugePages", clears stale direct sockets, builds the base image, starts native containers in dependency order, & launches `Camera` only after the required readiness conditions are satisfied.

### 18.5 Interactive Viewer

For NON-QUALITY, associate the browser with the exposed `User` "HTTP" endpoint before confirming the launcher prompt. Once connected, `Camera` begins the 300-frame source sequence & the browser can issue pose commands through buttons or keyboard controls.

### 18.6 Inspect the Active Nodes

```bash
sudo docker ps
sudo docker logs camera
sudo docker logs sff1
sudo docker logs sff2
sudo docker logs encoder
sudo docker logs decoder
sudo docker logs sff3
sudo docker logs user
```

### 18.7 Stop the Experiment

```bash
sudo ./stop_all.sh
```

The shutdown script removes all seven containers, clears direct "DPDK" socket / runtime files, releases "HugePages", synchronises persistent state, & drops caches. It deliberately does not reverse `isolcpus`, because "CPU" isolation is a reboot-tier experimental condition.

---

## 19. Entrypoint Execution Model

Every native node is compiled / launched from its mounted `/app` source tree rather than from a stale binary embedded in the image. `start_microservices.sh` supplies a node-specific `DPDK_CORE`, cpuset, optional "GPU" permission, & optional `QUALITY_CAPTURE` environment.

The `User` entrypoint additionally owns the auxiliary presentation / quality lifecycle:

```
QUALITY_CAPTURE = 0
  -> start "Python" "HTTP" / "WebSocket" bridge on Core 2 with nice +5
  -> start User "DPDK" on Core 0

QUALITY_CAPTURE = 1
  -> start a background shell gate waiting for /tmp/sfc-user-quality
  -> start User "DPDK" on Core 0
  -> after User "EOS" / capture serialization, launch serial gauge.py on Core 0
  -> Gauge writes /tmp/sfc-user-done
  -> User exits its quality wait & prints the final "EOS" state
```

The BLAS / numeric thread environment for `Gauge` is fixed to one thread (`OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`, `MKL_NUM_THREADS`, & `NUMEXPR_NUM_THREADS` all equal `1`). This prevents the offline assessment from silently spawning a larger "CPU" pool.

---

## 📊 20. Native Telemetry Files

The final repository produces a complete per-node observation chain:

| Component | File | Final column count |
|---|---|---:|
| Converter | `log/converter/telemetry_converter.csv` | `8` |
| `Camera` | `log/camera/telemetry_camera.csv` | `30` |
| `SFF1` | `log/sff1/telemetry_sff1.csv` | `42` |
| `SFF2` Route 0 | `log/sff2/telemetry_sff1_enc.csv` | `42` |
| `Encoder` | `log/encoder/telemetry_encoder.csv` | `69` |
| `SFF2` Route 1 | `log/sff2/telemetry_enc_dec.csv` | `42` |
| `Decoder` | `log/decoder/telemetry_decoder.csv` | `61` |
| `SFF2` Route 2 | `log/sff2/telemetry_dec_sff3.csv` | `42` |
| `SFF3` | `log/sff3/telemetry_sff3.csv` | `40` |
| `User` | `log/user/telemetry_user.csv` | `47` |
| `Encoder` "codec" | `log/encoder/ffmpeg.txt` | `vstats` text |
| `Decoder` "codec" | `log/decoder/ffmpeg.txt` | `vstats` text |

### 20.1 Exact CSV Schema Reference

**`Camera` — 30 columns**

```text
frame_id;selected;tx_complete;current_skip;last_control_frame;temporal_control_ms;camera_send_timestamp;tx_start_timestamp;tx_points;tx_packets;payload_bytes;reference_size_bytes;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;disk_io_ms;serialization_ms;tx_duration_ms;active_tx_ms;active_process_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```

**`SFF1` — 42 columns**

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;geometry_aggregation_ms;max_r_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;camera_node_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

**`SFF2` Routes 0 / 1 / 2 — 42 columns each**

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_media_bytes;tx_media_bytes;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;camera_node_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

**`Encoder` — 69 columns**

```text
frame_id;rx_complete;tx_complete;current_skip;event;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;codec_exit_time;node_exit_timestamp;original_points;rx_points;processed_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;conversion_ms;geometry_aggregation_ms;max_r_ms;projection_ms;codec_write_ms;active_tx_ms;active_process_ms;reference_process_ms;total_processing_ms;total_residency_ms;reference_residency_ms;node_efficiency_pct;reference_efficiency_pct;gpu_transfer_ms;gpu_kernel_ms;gpu_packing_ms;gpu_copyback_ms;host_overhead_ms;camera_node_ms;e2e_latency_ms;reference_e2e_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;reference_jitter_ms;raw_queue_ms;render_queue_ms;workload_ewma_ms;workload_ratio;frame_backlog;codec_backlog;encode_service_ms;encode_h265_ms;mse_y;psnr_y;ssim_y;mpeg_bytes_generated;ffmpeg_write_calls;ffmpeg_write_eagain;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```

**`Decoder` — 61 columns**

```text
frame_id;rx_complete;tx_complete;current_skip;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_media_bytes;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbps;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;arrived_points;eroded_points;valid_points;erosion_ms;reconstruction_ms;pose_ms;reconstruction_pipeline_ms;tx_duration_ms;active_tx_ms;active_process_ms;reference_process_ms;total_processing_ms;total_residency_ms;reference_residency_ms;node_efficiency_pct;reference_efficiency_pct;gpu_transfer_ms;gpu_copyback_ms;host_overhead_ms;camera_node_ms;e2e_latency_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;pose_control_ms;codec_queue_ms;frame_queue_ms;codec_backlog;decode_service_ms;decode_h265_ms;ffmpeg_write_calls;ffmpeg_write_failures;codec_queue_drops;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets;mbuf_starvation
```

**`SFF3` — 40 columns**

```text
frame_id;rx_complete;tx_complete;current_skip;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;rx_points;tx_points;rx_packets;tx_packets;payload_bytes;reference_size_bytes;data_integrity_pct;internal_throughput_mbs;reference_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;reference_bitrate_mbps;tx_duration_ms;active_tx_ms;active_process_ms;cycle_ms;header_wait_ms;total_residency_ms;node_efficiency_pct;reference_efficiency_pct;camera_node_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;eth_errors;ipv4_errors;udp_errors;nsh_errors;tx_zero_accepts;tx_partial_accepts;tx_resubmit_calls;tx_resubmitted_packets
```

**`User` — 47 columns**

```text
frame_id;rx_complete;current_skip;yaw;pitch;zoom;camera_send_timestamp;recv_start_timestamp;node_exit_timestamp;original_points;arrived_points;eroded_points;valid_points;rx_points;rx_packets;payload_bytes;data_integrity_pct;internal_throughput_mbs;logical_bitrate_mbps;network_bitrate_mbps;arrival_pct;erosion_pct;valid_pct;web_publish_ms;web_ack_ms;active_process_ms;total_residency_ms;node_efficiency_pct;camera_node_ms;e2e_latency_ms;reference_e2e_ms;schedule_delay_ms;instant_jitter_ms;desynced_jitter_ms;cmd_id;reference_cmd_ms;cmd_apply_ms;cmd_photon_ms;quality_save_ms;mean_error;geom_rmse;chamfer;hausdorff;mean_mm;rmse_mm;chamfer_mm;hausdorff_mm
```

### 20.2 "FFmpeg" `vstats` Field Semantics

The `Encoder` & `Decoder` `ffmpeg.txt` files are diagnostic "codec" chronologies, not one-to-one application-frame logs. The final archives each contain 396 statistics rows because persistent pre-roll, authentic application material, & drain / post-roll operations share the "codec" timeline. Native CSV `frame_id` attribution remains authoritative for the 300 application frames.

Accordingly, `vstats` rows must not be naively joined by ordinal position to `telemetry_encoder.csv` or `telemetry_decoder.csv`. "codec"-level rate / frame counters complement the native telemetry but occupy a distinct measurement domain.

### 20.3 Timing Quantities Must Not Be Added Indiscriminately

Per-node timings intentionally overlap. `active_process_ms`, `total_residency_ms`, "codec" service time, "GPU" sub-stages, source-to-node latency, & browser "CTP" answer different questions. Adding them as though they were independent serial stages would double-count concurrent / nested work.

The most appropriate end-to-end frontier is the timestamp already propagated from `Camera` & resolved by each terminal node. Browser `cmd_photon_ms` is further distinct because it ends only after the matching point cloud has been rendered.

### 20.4 Quality Columns Are Mode-Dependent

`mse_y`, `psnr_y`, & `ssim_y` are populated only when `Encoder` quality capture is active. Likewise `mean_error`, `geom_rmse`, `chamfer`, `hausdorff`, & their millimetre forms are merged by `Gauge` after a quality run. Interactive runtime telemetry intentionally leaves these fidelity fields unset.

### 20.5 Native Telemetry Serialization & Measurement Isolation

Native per-frame telemetry is accumulated within bounded in-memory structures throughout the active stream & is serialised to the corresponding `.csv` artefacts only after the terminal condition has been resolved. Consequently, native CSV file I / O is excluded from the 300-frame real-time measurement path. The quality mechanisms preserve the identical principle: `Encoder` retains "luma" references / attributed compressed material in memory, while `User` retains complete reconstructed snapshots in `quality_buffer`; persistent quality artefacts & objective evaluation are deferred until after "EOS".

The `ffmpeg.txt` files constitute a deliberate exception in **measurement domain**, not in CSV semantics. They are "FFmpeg"-originated diagnostic chronologies associated with the persistent "codec" processes & remain analytically separate from the authoritative native per-frame CSVs. They must therefore neither be interpreted as application-frame tables nor be used to redefine the native timing frontiers documented above.

---

## 🧪 21. Relevant Outcomes from the Validated Snapshot

Two final 300-frame archives are used deliberately for different questions. The NON-QUALITY run validates the complete interactive data / control path with the browser enabled. The QUALITY run removes Web presentation & appends post-stream coding / geometry fidelity indicators. They must not be merged into a single statistical population.

### 21.1 Dataset & Streaming Population

Both conditions operate upon the same 300-frame "Loot" binary sequence containing `238,146,391` original points. The final `User` reconstruction populations are:

| Quantity | NON-QUALITY | QUALITY |
|---|---:|---:|
| Original points | `238,146,391` | `238,146,391` |
| Arrived reconstruction candidates | `49,530,359` | `49,523,055` |
| Eroded points | `46,561,197` | `46,557,250` |
| Valid reconstructed points | `43,620,959` | `43,620,959` |

The final valid population is identical across both modes: `43,620,959` points, corresponding to approximately `18.317 %` of the original source population. The intermediate `arrived_points` & post-erosion `eroded_points` totals differ slightly between validation conditions, whereas the stricter final reconstruction population remains identical.

### 21.2 Complete-Chain Integrity

| Node / Route | NON-QUALITY | QUALITY |
|---|---|---|
| `Camera` | `300 / 300` admitted / `300 / 300` Tx | `300 / 300` admitted / `300 / 300` Tx |
| `SFF1` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` |
| `SFF2 Route 0` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` |
| `Encoder` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` |
| `SFF2 Route 1` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` |
| `Decoder` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` |
| `SFF2 Route 2` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` |
| `SFF3` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` | `300 / 300` Rx / `300 / 300` Tx, `100.0 %` |
| `User` | `300 / 300` Rx complete, `100.0 %` | `300 / 300` Rx complete, `100.0 %` |

Therefore the final primary route is complete in both conditions:

```
300 Camera frames admitted
300 frames received / forwarded by every service boundary
300 encoded application frames attributed
300 decoded frames reconstructed
300 final User frames complete
```

No final `SFF1`, `SFF2`, `Encoder`, `Decoder`, or `SFF3` primary Tx path reports partial accepts, exhausted retries, or `mbuf` starvation in either representative archive. `Camera` preserves complete source transmission despite its separately reported local zero-accept pressure, while `User` is a terminal receive endpoint & therefore exposes `rx_complete` rather than an application-frame `tx_complete` predicate.

### 21.3 Camera — Nominal 30-fps Source Operation & Local Backpressure

Measured source intervals are:

```text
NON-QUALITY mean interval = 33.337 ms -> 29.997 frames/s
QUALITY     mean interval = 33.336 ms -> 29.997 frames/s
```

For NON-QUALITY, `Camera` processing exhibits approximately:

| Metric | Mean | Median | P95 | Max |
|---|---:|---:|---:|---:|
| `disk_io_ms` | `4.484` | `4.511` | `5.298` | `6.164` |
| `serialization_ms` | `2.674` | `2.611` | `3.316` | `3.756` |
| `tx_duration_ms` | `7.964` | `7.991` | `8.543` | `9.096` |
| `active_process_ms` | `15.122` | `15.072` | `16.084` | `17.536` |

`Camera` alone encounters substantial local Tx zero-accept pressure while still preserving 300 / 300 completion. The final counts are:

| Condition | Frames with zero accepts | Zero-accept sum | Maximum / frame | Re-presented packets | Partial accepts | `mbuf` starvation |
|---|---:|---:|---:|---:|---:|---:|
| NON-QUALITY | `209` | `146,002` | `1,670` | `4,553,699` | `0` | `0` |
| QUALITY | `300` | `624,223` | `3,003` | `19,801,505` | `0` | `0` |

These counts describe repeated local `rte_eth_tx_burst()` presentation attempts. They are **not "UDP" retransmissions**, & they do not imply data-plane loss because frame completion remains 300 / 300.

### 21.4 SFF1 / "GAC" — In-Path Geometry Cost

In NON-QUALITY:

```text
geometry_aggregation_ms mean = 3.751 ms
max_r_ms                mean = 1.854 ms
active_process_ms       mean = 7.606 ms
total_residency_ms      mean = 13.982 ms
```

The entire primary input / output population remains at 100 % network integrity while the "GAC" performs packet-progressive aggregation & exact frame-completing radius evaluation.

### 21.5 SFF2 — Three Validated Proxy Transitions

The three route mean active costs in NON-QUALITY are:

```text
Route 0 SFF1 -> Encoder    : 5.083 ms
Route 1 Encoder -> Decoder : 0.018 ms
Route 2 Decoder -> SFF3    : 0.572 ms
```

Route 1 is predominantly compressed-media relay, Route 2 is reconstructed-point re-encapsulation, & Route 0 additionally handles the geometric service context. All three preserve 300 / 300 frames & 100 % integrity.

### 21.6 Encoder — Geometry Offload, "GPU", Workload Control, & "H.265"

NON-QUALITY representative values are:

| Metric | Mean | Median | P95 | Max |
|---|---:|---:|---:|---:|
| `conversion_ms` | `3.552` | `3.521` | `3.765` | `11.177` |
| `projection_ms` | `5.015` | `4.956` | `5.359` | `12.857` |
| `codec_write_ms` | `6.679` | `6.599` | `7.069` | `8.022` |
| `encode_service_ms` | `10.284` | `9.908` | `10.790` | `68.690` |
| `encode_h265_ms` | `76.487` | `76.856` | `78.025` | `78.555` |
| `workload_ratio` | `0.152` | `0.150` | `0.154` | `0.386` |

`frame_backlog` remains `0`, `ffmpeg_write_eagain` remains `0`, & `mbuf_starvation` remains `0`. The workload ratio never reaches the configured overload threshold, so all 300 frames retain `current_skip = 1`. Application-attributed compressed output exceeds `10.34 MB` over 300 frames in both final modes.

### 21.7 Decoder — Hardware Decode & Reconstruction

NON-QUALITY:

| Metric | Mean | Median | P95 | Max |
|---|---:|---:|---:|---:|
| `erosion_ms` | `0.183` | `0.177` | `0.220` | `0.546` |
| `reconstruction_ms` | `0.105` | `0.104` | `0.107` | `0.123` |
| `pose_ms` | `0.065` | `0.065` | `0.067` | `0.079` |
| `reconstruction_pipeline_ms` | `2.067` | `2.029` | `2.330` | `2.965` |
| `decode_service_ms` | `33.340` | `33.105` | `36.670` | `72.126` |
| `decode_h265_ms` | `125.476` | `125.144` | `128.393` | `164.501` |
| `e2e_latency_ms` | `228.718` | `228.177` | `231.976` | `267.590` |

The median "codec"-queue delay is only `0.046 ms`; there are no "codec" queue drops, "FFmpeg" write failures, or downstream frame losses.

### 21.8 SFF3 & User Terminal Delivery

`SFF3` primary forwarding remains light relative to the "codec" stages:

```text
SFF3 active_process_ms mean  = 1.083 ms
SFF3 total_residency_ms mean = 1.438 ms
```

The NON-QUALITY `User` end-to-end distribution is:

```text
mean   = 241.237 ms
median = 240.733 ms
P95    = 250.250 ms
max    = 277.330 ms
```

The browser returns positive frame acknowledgments for `96` rendered frames over a `Camera` timestamp span of approximately `9.968 s`, corresponding to approximately `9.63` acknowledged renders/s. This is explicitly a **viewer consumption rate**, not the 30-fps native data-path rate; all 300 `User` frames remain successfully received.

### 21.9 Interactive "Pose" / "Command-to-Photon" Results

The final NON-QUALITY run contains 42 command identifiers matched to returning pose states. The timing populations are:

| Frontier | Samples | Mean ( ms ) | Median ( ms ) | P95 ( ms ) | Max ( ms ) |
|---|---:|---:|---:|---:|---:|
| Reference command | `42` | `13.778` | `12.885` | `28.391` | `37.422` |
| Applied command | `42` | `34.728` | `34.761` | `52.165` | `74.846` |
| Command-to-Photon | `41` | `184.366` | `187.000` | `257.000` | `292.000` |
| `Decoder` pose-control | `42` | `21.573` | `23.108` | `35.329` | `63.556` |

`cmd_photon_ms` contains 41 positive values because one final matched command does not receive a browser "CTP" acknowledgment before the experiment closes. This distinction is retained rather than silently imputing a value.

### 21.10 Objective Encoder Quality

The 300-frame quality run produces complete "luma" indicators:

| Metric | Samples | Mean | Median | P95 | Max |
|---|---:|---:|---:|---:|---:|
| `MSE-Y` | `300` | `0.242` | `0.220` | `0.330` | `0.340` |
| `PSNR-Y` ( dB ) | `300` | `54.356` | `54.655` | `55.100` | `55.210` |
| `SSIM-Y` | `300` | `0.997356` | `0.998198` | `0.998588` | `0.998692` |

These values measure the custom projected "luma" representation, not a standards-compliant point-cloud "codec" rate-distortion curve.

### 21.11 Objective Reconstructed Geometry Quality

`Gauge` successfully evaluates all 300 complete quality frames:

| Metric | Mean | Median | P95 | Max | Unit |
|---|---:|---:|---:|---:|---|
| Mean geometric error | `6.392` | `6.137` | `8.295` | `8.834` | `mm` |
| Geometric RMSE | `6.249` | `6.108` | `7.150` | `7.893` | `mm` |
| Symmetric Chamfer | `11.164` | `10.883` | `13.029` | `13.583` | `mm` |
| Symmetric Hausdorff | `121.113` | `104.598` | `260.253` | `520.389` | `mm` |

The relatively larger Hausdorff tail records sparse worst-case spatial deviations & should therefore be interpreted alongside mean / RMSE / Chamfer rather than in isolation.

### 21.12 Quality-Capture Runtime Cost

The final stable `User` quality capture retains the reconstructed frame in the ordinary `web_points` region & copies only complete frames into the 1-GiB capture buffer. The measured `quality_save_ms` distribution is:

```text
mean   = 3.779 ms
median = 2.989 ms
P95    = 9.715 ms
P99    = 16.305 ms
max    = 22.425 ms
```

The maximum of `22.425 ms` remains below the nominal `33.333 ms` source period in this representative run, & the final `SFF3` -> `User` path reports no Tx zero-accept pressure. Objective `Gauge` computation itself begins only after "EOS" & therefore is excluded from these stream-time values.

`quality_save_ms` is intentionally retained as an independent instrumentation cost rather than being folded into `User` `node_exit_timestamp`, `e2e_latency_ms`, `total_residency_ms`, or `active_process_ms`. In the quality path, the native frame-ready frontier remains the terminal timing reference while the subsequent complete-frame memory copy is exposed separately; the two quantities should therefore be interpreted jointly when evaluating capture pressure, but not silently summed into a redefined end-to-end metric.

### 21.13 Active-Processing Statistical View — NON-QUALITY

| Node / Route | Mean ( ms ) | Median ( ms ) | P95 ( ms ) | Max ( ms ) |
|---|---:|---:|---:|---:|
| `Camera` | `15.122` | `15.072` | `16.084` | `17.536` |
| `SFF1` | `7.606` | `7.572` | `8.076` | `8.253` |
| `SFF2 Route 0` | `5.083` | `5.114` | `5.416` | `5.860` |
| `Encoder` | `15.246` | `15.107` | `15.980` | `30.369` |
| `SFF2 Route 1` | `0.018` | `0.015` | `0.056` | `0.086` |
| `Decoder` | `36.711` | `36.401` | `40.476` | `75.420` |
| `SFF2 Route 2` | `0.572` | `0.549` | `0.751` | `0.926` |
| `SFF3` | `1.083` | `1.090` | `1.204` | `1.324` |
| `User` | `4.057` | `2.978` | `9.962` | `18.746` |

The table is not an additive latency decomposition: several values represent nested or asynchronous work, especially around the persistent "codec" processes.

### 21.14 "FFmpeg" Cross-Check

Both final `Encoder` & `Decoder` `ffmpeg.txt` archives contain 396 `vstats` rows. These rows include private warm-up / drain chronology in addition to authentic application frames. The count therefore must not be described as `15 + 300`, nor should it be used as a surrogate for native 300-frame completion. Application CSVs remain authoritative for frame identity & correctness.


### 21.15 Direct Comparison with the Application-Level Reference Baseline

A direct comparison was performed against the supplied 300-frame **probe-disabled GPU reference logs** associated with the Lacaria pipeline & against the 10-Mbit/s quality values reported in the reference thesis. The comparison is intentionally restricted to frontiers that can be reconstructed with compatible semantics. It is not presented as a same-binary controlled A / B experiment: transport, node boundaries, warm-up strategy, "GPU" kernels, control placement, & some quality-analysis details differ.

For end-to-end responsiveness, the reference log permits an explicit Camera-to-Client calculation:

```text
reference_e2e_ms = ( t_client_recv_abs - t_camera_send ) * 1000
```

Across all 300 supplied reference rows, this produces:

```text
Reference median Camera -> Client reception = 341.785 ms
Current median Camera   -> User frame-ready = 240.732 ms
Relative median reduction                   = 29.6 %
```

The current value uses `User.reference_e2e_ms`, deliberately terminating at the native complete-frame frontier rather than at browser rendering. The median is selected because the supplied reference trace contains both a pronounced startup transient & a late stall; it is therefore more robust than comparing raw full-run means. The current archive simultaneously retains 300 / 300 complete native frames & an approximately `29.997 frames/s` source cadence. The supported conclusion is consequently a **lower representative end-to-end latency at the same 300-frame / nominal-30-fps service objective**, not a claim of universally lower latency under every workload.

Selected application-stage comparisons additionally expose where the paradigm shift redistributes work:

| Semantically Related Frontier | Reference Median | Current Median | Relative Change | Interpretation |
|---|---:|---:|---:|---|
| Encoder point conversion | `8.667 ms` | `3.521 ms` | `-59.4 %` | Current point conversion is packet-progressive; part of geometric preparation is externalised upstream |
| Encoder projection | `10.882 ms` | `4.956 ms` | `-54.5 %` | Encoder-local projection is materially shorter, but `SFF1` now performs real geometric work & must not be treated as zero-cost |
| Post-decode unpack / erosion / reconstruction-render work | `6.108 ms` | `2.029 ms` | `-66.8 %` | Reference value is `unpack_ms + erode_ms + render_ms`; current value is `Decoder.reconstruction_pipeline_ms`, so the comparison is indicative rather than bit-identical |

These reductions are **not** equivalent to a statement that every node became faster. The service boundaries changed: for example, current `Encoder.total_residency_ms` includes persistent asynchronous "codec" chronology under a different timing frontier, while geometry previously hidden within application processing is now explicitly charged to `SFF1`. The defensible advantage is therefore that selected computational frontiers & the end-to-end path are lower while costs are distributed into independently measurable functions.

#### Fidelity Relative to the Reference

The decomposition preserves the same source sequence exactly at the population level. The mean source population is `793,821.303` points / frame in both data sets. More significantly, the final valid reconstructed population is almost identical:

```text
Reference mean valid_points = 145,403.233
Current mean valid_points   = 145,403.197
Difference                  = 0.037 points / frame
```

When the two 300-frame sequences are aligned by ordinal source frame, `292 / 300` final `valid_points` counts are exactly equal & the largest absolute difference is only `10` points. The mean final-population ratio is approximately `18.317 %` of the original cloud in both implementations. This is strong evidence that service decomposition & additional `SFF` traversal do **not** introduce systematic reconstruction-population loss.

Signal & geometric quality must be stated more carefully. At the common 10-Mbit/s condition, the reference thesis reports approximately `MSE-Y = 0.129`, `PSNR-Y = 56.06 dB`, & `SSIM-Y = 0.997`; the current 300-frame quality run reports `0.242`, `54.356 dB`, & `0.997356`, respectively. Thus, structural similarity remains effectively in the same high-fidelity regime, while `PSNR-Y` is approximately `1.704 dB` lower & `MSE-Y` is higher. Exact "luma"-fidelity invariance is therefore **not** claimed.

An additional comparability constraint originates from the execution environment itself. The current `"CUDA"` kernels are compiled explicitly for `sm_61`, while the host-side build additionally enables `-ffast-math`; the resulting numerical path is therefore tied to a specific compilation target rather than being guaranteed to reproduce bit-identical floating-point operations across heterogeneous `"GPU"` architectures. At the same time, the compressed representation is produced by the hardware `"NVENC"` path, whose concrete execution environment includes the `"GPU"` generation, NVIDIA driver, `"FFmpeg"` build, & encoder implementation available during the run.

The observed `PSNR-Y` / `MSE-Y` difference must consequently not be interpreted as a distortion introduced by `"SFC"` or `"UDP"` alone. The present comparison changes several variables simultaneously, including service decomposition, `"CUDA"` compilation / execution conditions, persistent `"codec"` handling, & the hardware-software video stack. Establishing causal attribution for the residual quality difference would require rebuilding & executing both architectures under an identical `"GPU"`, driver, `"CUDA"` toolchain, `"FFmpeg"` / `"NVENC"` configuration, source representation, & metric implementation.

The same caution applies to robust post-erosion geometry. Converting the reference thesis's 10-Mbit/s robust values through the common `1 voxel = 1.820 mm` scale gives:

| Metric | Reference Robust Post-Erosion | Current `Gauge` | Difference in Interpretation |
|---|---:|---:|---|
| Mean geometric error | `4.472 mm` | `6.392 mm` | Current value is higher; both remain millimetre-scale |
| Geometric RMSE | `5.586 mm` | `6.249 mm` | Same order of magnitude |
| Symmetric Chamfer | `10.025 mm` | `11.164 mm` | Same order of magnitude |
| Symmetric Hausdorff | `87.364 mm` | `121.113 mm` | Current worst-case tail is higher |

The scientifically supportable statement is therefore **fidelity preservation at the same high-quality operating regime, not numerical identity**. Network / service decomposition preserves all 300 frames, the final reconstructed population is essentially unchanged, `SSIM-Y` remains approximately `0.997`, & geometric errors remain comparable in scale; however, the current quality metrics are not universally equal to or better than the reference. A claim that "quality is unaltered" in the strict numerical sense would exceed the evidence because the implementations do not constitute an isolated transport-only A / B experiment.

#### Orchestration & Request-Handling Consequences

The reference thesis explicitly identifies its monolithic `Encoder` & Client placement as an orchestration limitation & proposes splitting projection / encoding & decoding / client processing into independently deployable blocks. The current architecture directly advances that objective by exposing seven native graph-level placement units instead of four coarse services. This is a `75 %` increase in **placement granularity**, not a 75 % resource saving.

The control model also removes dedicated transport-connection state from the native service plane. The reference architecture maintains two persistent "TCP" feedback channels, each bound to a specific upstream application node. The current design instead classifies two fixed-size control datagrams into independent `SPI 200` & `SPI 300` service paths. A new request family can consequently be represented through a new classifier / service-path contract without requiring `Encoder` / `Decoder` to acquire "SFC" parsing logic or introducing another byte-stream framing scheme into the `SFF` path.

This improves **orchestration flexibility**, but present evidence does not yet establish a maximum simultaneous-user or request-rate gain. The reference thesis itself notes that its proposed Density Test would replicate the complete four-node stack because the original application has no stream identifier & accepts one connection per socket. The current implementation likewise remains a single-stream experimental chain; it has not yet executed an N-user density test. Its demonstrated advantage is finer placement / steering granularity & O(1)-sized latest-state control, while quantitative multi-stream capacity remains future work rather than a claimed result.

#### Presentation-Level Claim Boundaries

For thesis slides, the comparison should be framed through four compact messages rather than through an excessively tall topology: ( i ) the reference `Encoder` / Client blocks are decomposed into network-visible functions; ( ii ) the complete primary route preserves 300 / 300 application frames; ( iii ) representative median end-to-end latency decreases from `341.785 ms` to `240.732 ms` under the compatible application-level frontier; & ( iv ) fidelity remains in the same high-quality regime, but strict numerical equality is not claimed because the "GPU", driver, "CUDA", "FFmpeg" / "NVENC", persistent "codec" handling, & metric paths are not a controlled transport-only A / B condition. This wording preserves the scientific advantage of the work without overstating the quality comparison.

---

## ⚠️ 22. Experimental Interpretation & Known Boundaries

### 22.1 Logical Bytes vs. Wire Bytes

Logical payload, full application datagram, & reference-size bitrate answer different questions. "Ethernet" framing, service metadata, "UDP" / "IP" headers, reconstructed-point payloads, & compressed-media bytes must not be interchanged when reporting throughput.

### 22.2 `CACHE_MODE` & `WARM_MODE` Are Part of the Experimental Condition

The final source condition is:

```text
CACHE_MODE = CACHE_MODE_MIDDLE
WARM_MODE  = WARM_MODE_ENABLED
```

Changing either value defines a different benchmark.

### 22.3 Two Validation Modes Must Remain Separate

NON-QUALITY measures the interactive runtime system & includes the Web bridge / browser. QUALITY removes that presentation path & activates in-memory fidelity capture followed by post-stream analysis. Browser "CTP" must therefore never be inferred from a quality run, while PSNR / SSIM / `Gauge` geometry must never be attributed to a NON-QUALITY run.

### 22.4 Source Scheduling, Descriptor Depth, & Backpressure Are Joint Variables

The final 4096-descriptor configuration is a fixed experimental condition. `Camera` zero-accept volume demonstrates that a complete 300-frame run can still contain substantial local producer / consumer pressure. Changing queue depth, retry bounds, affinity, or source-cache behaviour alters the operating point.

### 22.5 Core Affinity Is Part of the Experiment

All eight logical cores are used by the final application / "codec" layout across the stream phase. The former "OVS" "PMD" core is not an independent hidden resource; it has been reassigned to application work. "SMT" sibling sharing & Core `0` housekeeping must be retained in comparative reports.

### 22.6 Complete Route 2 Is Validated

Unlike earlier snapshots, `Decoder -> SFF2 -> SFF3 -> User` now possesses a stable `dec_hdr` contract, exact point accounting, route-specific telemetry, service re-encapsulation, final decapsulation, & 300 / 300 completion in both archived validation modes.

### 22.7 End-to-End & "CTP" Are Available but Distinct

Native `User` `e2e_latency_ms` measures the `Camera` timestamp to `User` node exit. Browser `cmd_photon_ms` measures a control request to the first actually rendered matching frame. These are different causal frontiers & must not be summed or substituted for one another.

### 22.8 "NSH" Interoperability Is Not Claimed

The project deliberately uses an "NSH"-inspired closed contract. The experimental metadata class / next-protocol choices do not establish generic interoperability with arbitrary RFC 8300 implementations.

### 22.9 The "Temporal" Controller Is Integrated but Remains Inactive in the Final Representative Runs

`TEMPORAL_ADAPTATION_ENABLED` is active throughout both final representative archives. Under these specific NON-QUALITY & QUALITY operating conditions, however, the measured workload remains below the configured overload thresholds & `current_skip = 1` throughout the 300-frame sequence. Consequently, the absence of `SKIP+1` / recovery events in these two archives must not be interpreted as an unimplemented or unvalidated control mechanism; it indicates that the final stable operating point does not require source-rate reduction. The controller, its re-presentation semantics, hysteresis, & returned-state confirmation remain part of the implemented architecture, while the final archives deliberately characterise the nominal non-overloaded condition.

### 22.10 "Codec" Diagnostics Are Not Application Frame Tables

Persistent pre-roll / post-roll deliberately injects private chronology into `ffmpeg.txt`. Native application CSV attribution must remain authoritative.

### 22.11 Objective Geometry Depends upon the Stated Gauge Procedure

The millimetre metrics include pose reversal, statistical filtering, robust "ICP" alignment, & nearest-neighbour comparison using `VOXEL_MM = 1.820`. They are therefore metrics of the documented evaluation pipeline, not unqualified raw point-index differences.

### 22.12 Dataset Artefacts Are External to Git

The repository intentionally excludes the multi-gigabyte PLY / BIN sequence. Reproducibility requires independent dataset acquisition plus the exact Converter settings.

### 22.13 Reference Comparison Is a Cross-Architecture Comparison

The direct values in Section 21.15 combine the current archived runs, the supplied probe-disabled reference telemetry, & published values from the reference thesis. They are useful because the dataset, target cadence, projection family, hardware codec family, & 10-Mbit/s quality condition are closely related, but they do not form a transport-only controlled A / B test. Median end-to-end improvement can be reported as an observed cross-implementation result; causal attribution must remain distributed across transport, buffering, function decomposition, "GPU" kernels, persistent "codec" handling, affinity, & control placement.

In particular, a lower current `Encoder` sub-stage does not imply that the corresponding work disappeared: part of the geometry is now explicitly executed by `SFF1`. Likewise, the absence of a compatible per-core utilisation log in the supplied reference material prevents a defensible claim that the current architecture consumes a specific percentage less total "CPU".

### 22.14 "UDP" Applicability Is Deliberately Restricted

The current "UDP" rationale is valid for the closed "DPDK" / "SFC" experiment because message identity, application pacing, returned-state confirmation, queue pressure, & integrity are explicitly controlled. "RFC 8085" should be treated as a boundary condition rather than ignored: a deployment over the unrestricted Internet would require an appropriate congestion-control / fairness strategy, checksum policy, path-"MTU" consideration, security treatment, & potentially a different transport. The project therefore claims **semantic suitability within its controlled domain**, not universal transport superiority over "TCP".

---

## 🛠️ 23. Main Engineering Challenges & Current Solutions

### Limited Logical Cores

**Challenge:** Seven native nodes, two persistent "codec" processes, a browser bridge, & host housekeeping must coexist on eight logical "CPU" threads.

**Current approach:** Remove the previous "OVS"-"DPDK" intermediary, pin every native role explicitly, separate `Decoder` "DPDK" from "codec" I/O, share only comparatively light / asynchronous roles, lower "Python" bridge priority, & run `Gauge` after "EOS" rather than concurrently.

### "DPDK" Backpressure at the Source

**Challenge:** The `Camera` can encounter repeated zero-accept Tx returns even when the final stream remains complete.

**Current approach:** Bounded local re-presentation, explicit zero / partial counters, 4096-descriptor queues, & strict distinction between local ring acceptance & "UDP"-level semantics.

### Sustaining the Nominal 30-fps Operating Point

**Challenge:** Source disk access, serialisation, packet submission, in-path geometry, projection, encode, decode, reconstruction, & terminal delivery must coexist without progressive frame loss.

**Current approach:** Middle-cache / warm source condition, persistent "GPU" / "codec" state, direct adjacency, cooperative polling, bounded queues, & complete per-node telemetry. Both final modes sustain approximately 29.997 source frames/s with 300 / 300 final completion.

### Performing Useful Work In-Network

**Challenge:** Identify operations that are mathematically compatible with packet-progressive execution.

**Current approach:** Move associative sums / extrema into `SFF1`, retain the centroid-dependent exact radius boundary at frame completion, & expose validated geometry to `Encoder` through service metadata.

### Protecting Rx While Computing

**Challenge:** Long "CPU" / "GPU" / pipe waits can delay packet servicing.

**Current approach:** Incremental packet placement, cooperative "DPDK" polling, separate "codec" writer roles, persistent buffers, & bounded frame / "codec" queues.

### Maintaining Service State Around Unaware Applications

**Challenge:** `Encoder` & `Decoder` should remain ordinary application parsers.

**Current approach:** `SFF2` captures, advances, removes, & reconstructs service state around both unaware nodes; `SFF3` consumes the final aware primary state.

### Reliable Control Semantics Without a Reliable Byte Stream

**Challenge:** "Pose" & "Temporal" directives are operationally important, yet the native Data Plane deliberately avoids persistent "TCP" state. A lost, duplicated, or delayed control datagram must not leave the application permanently inconsistent.

**Current approach:** Commands encode absolute desired state rather than relative event histories. "Pose" retains one timestamp across re-presentations & `Decoder` suppresses duplicate / older timestamps; "Temporal" retains an absolute requested skip & is re-issued while the returning `Camera` stream has not reflected it. Both paths therefore verify application effect through returned state. This provides the required convergence semantics while preserving one-message / one-datagram classification at the `SFF` frontiers.

### Preserving "MTU" Across Different Traffic Types

**Challenge:** Raw points, geometric metadata, compressed "MPEG-TS" groups, reconstructed points, "Temporal" control, & "Pose" control possess different envelopes.

**Current approach:** Derive each payload limit from its complete datagram structure; the largest final primary aware datagram remains below 1500 B.

### Correctly Attributing Asynchronous "Codec" Output

**Challenge:** Pipe reads & "codec" output do not preserve source frame boundaries.

**Current approach:** Persistent pre-roll, frame-order attribution, "MPEG-TS" / "PES" parsing, explicit decoder frame ordering, private warm-up IDs, & native application CSVs separated from `vstats` chronology.

### Avoiding Browser Backlog

**Challenge:** Browser rendering cannot consume every native 30-fps point-cloud snapshot on the constrained test host.

**Current approach:** Latest-frame-only shared publication, peer queue capacity one, payload copy deferred until consumer readiness, one frame in flight, & render-triggered acknowledgment. Native `User` reception therefore remains 300 / 300 even when browser acknowledgments occur at a lower rate.

### Measuring Quality Without Polluting the Runtime Experiment

**Challenge:** Disk writes, "ICP", "KD-Tree" searches, & quality filters can materially disturb a real-time benchmark.

**Current approach:** Bounded in-memory capture during QUALITY, persistent serialization after "EOS", serial `Gauge` execution only after `User` readiness, & a separate NON-QUALITY run for interaction / "CTP". The retained `User` design intentionally stages reconstruction in `web_points` & copies only complete frames into `quality_buffer`; a direct-write capture variant was experimentally rejected after producing unacceptable host-level instability, so it is explicitly excluded from the validated measurement configuration.

---

## ✅ 24. Reproducibility Checklist

Every archived benchmark should preserve at least:

```text
source revision
README revision
host kernel
"CPU" model & "SMT" topology
"GPU" model
NVIDIA driver
"CUDA" version
"DPDK" version
"Docker" version
"FFmpeg" version
"Python" / "NumPy" / "SciPy" / "websockets" versions

"HugePages" count & size
"Docker" cpusets
isolcpus state
native "DPDK" lcores
Encoder FFMPEG_CPU
Decoder FFMPEG_CPU
User bridge affinity / nice level
QUALITY_CAPTURE

Camera CACHE_MODE
Camera WARM_MODE
TARGET_FPS
K_FRAMES
POINTS_PER_PACKET
BURST_SIZE
MAX_ZERO_ACCEPTS
Rx / Tx descriptor depth
virtio queue_size

MAIN_SPI / SI transitions
TEMPORAL_SPI / SI
POSE_SPI / SI
DEFAULT_TTL / metadata class / next-protocol values
exact Main / Temporal / Pose payload sizes
control retry interval & returned-state confirmation rules
"UDP" checksum convention & controlled-environment applicability
SFF2 proxy rules
Route-0 / Route-1 / Route-2 byte semantics

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
QUALITY_STREAM_SIZE
target "H.265" bitrate / buffer
"GOP" / "IDR" configuration

Decoder QUEUE_SIZE
WRITE_BATCH_SIZE
I420_BUFFER_COUNT
hardware decode configuration
reconstruction / occupancy dimensions
pose-control policy

User QUALITY_BUFFER_SIZE
"WebSocket" compression state
peer queue size / one-frame-in-flight policy
Gauge VOXEL_MM / "ICP" / outlier parameters

all native telemetry CSV files
Encoder / Decoder ffmpeg.txt + stderr logs
comparison-baseline revision / log set when reporting cross-architecture percentages
```

The final representative configuration is:

```text
CACHE_MODE          = CACHE_MODE_MIDDLE
WARM_MODE           = WARM_MODE_ENABLED
OFFLOAD_MODE        = OFFLOAD_MODE_ENABLED
TEMPORAL_ADAPTATION = TEMPORAL_ADAPTATION_ENABLED
TARGET_FPS          = 30
K_FRAMES            = 300
POINTS_PER_PACKET   = 80
BURST_SIZE          = 32
MAX_ZERO_ACCEPTS    = 2048
Rx / Tx descriptors = 4096
User queue_size     = 4096
"HugePages"         = 1024 x 2 MiB
isolcpus            = 1-7, when enabled
```

Core placement:

```text
0 User "DPDK" / post-"EOS" Gauge in QUALITY
1 Camera
2 Decoder "FFmpeg" + writer / lower-priority Web bridge in NON-QUALITY
3 SFF1
4 Decoder "DPDK"
5 Encoder "DPDK"
6 SFF2
7 SFF3 + Encoder "FFmpeg" / writer
```

A benchmark attains reproducibility only when the result files & the exact mode / placement generating them are archived concurrently.

---

## 🚧 25. Future Extensions

The functional `Camera`-to-`User` chain is complete, including the `"Main"` service path, the reverse `"Temporal"` & `"Pose"` control paths, the quality condition, native per-frame telemetry, browser-side `"Command-to-Photon"` tracking, & the final validated CPU-affinity arrangement. Future work should therefore concern **broader deployment conditions**, **protocol interoperability**, **multi-stream scalability**, & **additional platform-level characterisation**, rather than completion or re-validation of mechanisms already consolidated within the present implementation.

### Portability & Deployment

```text
1. repeat the complete experiment on hosts exposing a different number of physical / logical cores
2. validate the final CPU-affinity policy across heterogeneous "SMT" / "NUMA" topologies
3. evaluate deployment across physically distributed hosts rather than the current controlled local topology
4. quantify the sensitivity of the validated service chain to heterogeneous compute resources across edge / cloud placement conditions
```

These experiments would establish how strongly the current operating point depends upon the characteristics of the reference host, while preserving the already validated service semantics, packet formats, & node responsibilities.

### Protocol Interoperability

```text
1. evaluate fully standards-compliant "NSH" encapsulation if interoperability with external "SFC" implementations becomes a research requirement
2. validate the closed packet representation on architectures exposing different host endianness / alignment characteristics
3. revisit checksum, congestion-control, & transport assumptions before any unrestricted wide-area or Internet-facing deployment
4. assess compatibility with external service-chain classifiers / orchestrators without altering the internal behaviour of "SFC"-unaware functions
```

The current packet representation remains intentionally `"NSH"-inspired` & closed within the experimental topology; interoperability with generic RFC-compliant service-chain infrastructures is therefore a distinct extension rather than a missing element of the present implementation.

### Multi-Stream Scalability & Orchestration

```text
1. introduce an explicit stream / session identifier if concurrent multi-user multiplexing is required
2. evaluate whether selected "SFC" functions can be shared across concurrent sessions while preserving per-stream service state
3. measure replicated-chain vs shared-function deployment strategies under controlled multi-stream load
4. quantify orchestration decisions in terms of placement granularity, resource assignment, service migration cost, & request admission capacity
```

The present architecture already exposes a finer placement granularity than the monolithic reference blocks; however, true multi-stream scalability requires explicit stream identity & controlled contention experiments before quantitative claims regarding concurrent-user capacity can be made.

### Extended Resource Characterisation

```text
1. complement the existing per-node latency / residency / efficiency telemetry with platform-level "CPU" utilisation measurements
2. introduce "GPU" utilisation & memory-bandwidth observations at the projection / reconstruction stages
3. measure power / energy consumption where suitable hardware instrumentation is available
4. relate resource consumption to placement decisions across different orchestration scenarios
```

This extension would allow the currently measured latency, throughput, & quality profiles to be complemented by a complete compute-resource cost model, enabling more direct use of the pipeline as an orchestration input within heterogeneous compute-continuum scenarios.

### Broader Application Validation

```text
1. repeat the validated pipeline on additional volumetric datasets exhibiting different point densities, motions, & spatial distributions
2. assess the portability of the fixed packetisation, codec, & reconstruction parameters across such datasets
3. evaluate the integration of an actual "VR" / "AR" terminal while preserving the existing browser / shared-memory diagnostic path as a reference condition
```

These experiments would test the generality of the current implementation beyond the `Loot` sequence without changing the architectural principles validated in the present work.

The immediate research priority is therefore no longer architectural completion or re-validation of the current service chain; it is the extension of an already complete & experimentally characterised `"SFC"` pipeline toward **portable deployment**, **standards-oriented interoperability**, **multi-stream orchestration**, & **broader resource-aware evaluation**.

---

## 📚 26. References


1. J. Halpern & C. Pignataro, **"Service Function Chaining" ( "SFC" ) Architecture**, "RFC 7665", IETF, 2015.
2. P. Quinn, U. Elzur, & C. Pignataro, **"Network Service Header" ( "NSH" )**, "RFC 8300", IETF, 2018. The present project adopts its "SPI" / "SI" terminology & architectural concepts but refrains from claiming complete wire-format interoperability.
3. J. Postel, **"User Datagram Protocol"**, "RFC 768", IETF, 1980. Used as the base datagram-format reference for the fixed-width application messages carried by the native Data Plane.
4. L. Eggert, G. Fairhurst, & G. Shepherd, **"UDP Usage Guidelines"**, "RFC 8085" / "BCP 145", IETF, 2017. The current deployment is explicitly scoped as a controlled environment; general-Internet congestion-control & checksum guidance remains an applicability boundary.
5. W. Eddy, Ed., **"Transmission Control Protocol" ( "TCP" )**, "RFC 9293", IETF, 2022. Used as the comparison reference for reliable byte-stream semantics, connection state, sequence / acknowledgment processing, & minimum header structure.
6. E. d'Eon, B. Harrison, T. Myers, & P. A. Chou, **8i Voxelized Full Bodies — A Voxelized Point Cloud Dataset**, ISO/IEC JTC1/SC29 Joint WG11/WG1 input document WG11M40059/WG1M74006, Geneva, January 2017.
7. **JPEG Pleno Database**, *8i Voxelized Full Bodies ( 8iVFB v2 ) — A Dynamic Voxelized Point Cloud Dataset*.
8. **"DPDK" Project**, Data Plane Development Kit documentation, including Ethdev, "virtio-user", & "vhost-user" interfaces.
9. **NVIDIA**, "CUDA" Toolkit documentation & NVIDIA Video Codec / "NVENC" / hardware-decoding documentation.
10. **"FFmpeg" Project**, "FFmpeg" documentation, filter reference, hardware acceleration, & `vstats` facilities.
11. **Three.js Project**, Three.js / WebGL point-geometry documentation used by the browser presentation frontier.
12. **SciPy Project**, `scipy.spatial.cKDTree` documentation employed by the offline geometric `Gauge`.
13. Maria Giovanna Lacaria, **Point Cloud Coding for Extended Reality Services**, Master's Thesis, Sapienza University of Rome, Academic Year 2025/2026. This reference constitutes the application-level architectural & evaluation baseline explicitly discussed by the project.

---

## Final Note


The principal scholarly contribution of this repository resides in the **holistic co-design of packet transport, service steering, computation, reconstruction, control, presentation, orchestration granularity, & measurement** under a deliberately constrained software data-plane environment.

The point cloud is not merely relayed between isolated applications. `Camera` governs source admission from an `Encoder`-derived workload signal; `SFF1` computes useful geometry while packets traverse the data path; `SFF2` preserves experimental service state around two unaware application functions; `Encoder` projects & compresses the geometry through persistent "GPU" / "codec" state; `Decoder` reconstructs the point cloud & applies independent `User` "Pose"; `SFF3` closes the aware primary chain & classifies the reverse "Pose" path; & `User` separates native reception from asynchronous browser presentation while retaining explicit command & quality telemetry.

The use of "UDP" is therefore not justified by an undifferentiated claim that it is simply faster than "TCP". It follows from the current protocol semantics: fixed-size application entities are directly classifiable by SFFs, every control request encodes absolute latest state, duplicates / stale commands can be rejected deterministically, & unresolved state is re-presented until its application effect is observed in the returning data stream. This design avoids introducing byte-stream reassembly / connection-control state into the native "DPDK" service plane while retaining a clear reliability mechanism at the application frontier. Its validity is intentionally restricted to the documented controlled environment.

The two final 300-frame "Loot" archives establish complementary outcomes. In both modes, the entire primary route preserves 300 / 300 frame completion with 100 % recorded data integrity at every native forwarding / application frontier & a `Camera` cadence of approximately 30 frames / s. The interactive run further exercises 42 matched "Pose" directives & browser `Command-to-Photon` acknowledgments, while the isolated quality run evaluates all 300 reconstructed frames & recovers complete `PSNR-Y`, `SSIM-Y`, mean / `RMSE` / `Chamfer` / `Hausdorff` geometry indicators.

Against the supplied probe-disabled application-level reference logs, the current native complete-frame frontier exhibits a median Camera-to-User latency of approximately `240.732 ms` versus `341.785 ms` for Camera-to-Client reception in the reference, corresponding to an observed median reduction of approximately `29.6 %`. Selected Encoder-local conversion / projection & post-decode reconstruction frontiers are likewise lower, while the added `SFF` costs remain explicitly measurable. These comparisons do not imply that every per-node residency is lower; function boundaries & asynchronous "codec" timing definitions have changed.

Fidelity is preserved in the more defensible sense of maintaining the same high-quality operating regime rather than reproducing every reference number exactly. The final valid reconstructed population is essentially invariant across the aligned 300-frame sequences (`145,403.233` versus `145,403.197` mean points / frame; 292 exact frame-wise population matches), & `SSIM-Y` remains approximately `0.997`. Some current "luma" / geometric error values are nevertheless higher than the reference 10-Mbit/s results; therefore exact quality invariance is deliberately **not** claimed without a controlled transport-only A / B experiment.

The orchestration contribution is similarly structural rather than rhetorical. The reference thesis identifies its monolithic `Encoder` & Client blocks as an obstacle to fine-grained placement & proposes their decomposition as future work. The current primary graph exposes seven native placement units rather than four coarse services — `75 %` more graph-level placement points — while the removal of one former switch-only "PMD" reservation makes `12.5 %` of this eight-thread host's logical scheduling capacity available for application placement. Neither percentage is presented as a measured reduction in total "CPU" consumption; the former quantifies granularity & the latter quantifies reallocation.

Consequently, the repository has progressed beyond an upstream proof of concept into a complete experimental `Camera`-to-`User` platform whose remaining limitations are primarily methodological: the packet format is intentionally "NSH"-inspired rather than a generic interoperable stack, the custom six-view "HEVC" representation is not a standards-compliant point-cloud "codec", the final representative archives deliberately exercise the nominal non-overloaded "Temporal" operating point, multi-user density remains unmeasured, & cross-reference resource percentages require a dedicated equal-hardware campaign.

The resulting platform is therefore best interpreted as an empirical study of **how a coarse application pipeline can be decomposed into explicit service functions, which volumetric-streaming operations benefit from in-path execution, how latest-state controls can remain reliable without importing a reliable byte stream into the packet path, how state can be retained around unaware functions, & how latency / fidelity / resource boundaries can be made sufficiently explicit for future orchestration decisions**.