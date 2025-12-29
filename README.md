# ns3-wifi-stats  
**Link-layer measurement framework for IEEE 802.11 simulations in ns-3**

This repository contains a set of ns-3 simulation scenarios developed to validate and
illustrate a link-layer measurement framework for IEEE 802.11 networks. The focus is on
collecting and analysing PHY- and MAC-layer statistics using helper-based instrumentation
available in ns-3 (version 3.44).

The scenarios are designed to support detailed investigation of contention-driven
behaviour in Wi-Fi networks, going beyond traditional network-layer metrics such as
throughput and packet loss.

---

## Measurement helpers used

The simulations make use of complementary helper components and trace-based mechanisms
provided by the ns-3 Wi-Fi module:

- **WifiPhyRxTraceHelper**  
  Used to collect PHY-layer reception statistics, including successful and failed PSDU
  receptions and frame-type classification (DATA, ACK, RTS, CTS, BlockAck).

- **WifiTxStatsHelper**  
  Provides MAC-layer transmission statistics at the MPDU level, including transmission
  attempts, retransmissions and retry-limit drops.

- **WifiCoTraceHelper**  
  Used to measure channel occupancy and airtime usage, independently of successful payload
  delivery.

- **Custom trace callbacks**  
  Additional MAC- and PHY-layer callbacks are used for timing-related measurements such as
  end-to-end delay, channel access delay, jitter and transmission timestamps.

The combination of these mechanisms allows direct correlation between PHY reception
outcomes, MAC retransmission behaviour and airtime consumption.

---

## Included simulation scenarios

The repository contains the following example scenarios:

- `wifi-stats.cc`  
  Basic validation scenario for an IEEE 802.11a ad-hoc network with a single transmitting
  station. Used to verify PHY/MAC timing, frame exchanges and measurement correctness under
  contention-free conditions.

- `wifi-stats-ax.cc`  
  Single-station IEEE 802.11ax validation scenario, including optional static Block
  Acknowledgement configuration. Used to compare PHY timing and transmission behaviour
  between legacy and modern Wi-Fi standards.

- `wifi-stats-pcoll.cc`  
  Multi-station IEEE 802.11ax scenario for estimating collision probability based on
  MAC-layer retransmissions and failures under saturated uplink traffic.

- `wifi-stats-delay.cc`  
  Scenario focused on delay, channel access delay and jitter measurements using custom
  timestamp-based instrumentation.

- `wifi-stats-perf.cc`  
  Saturated performance scenario analysing throughput, retransmissions and channel
  occupancy as a function of the number of active stations.

---

## Purpose and validation approach

The primary goal of this repository is to **validate the correctness and usability of
helper-based link-layer measurement mechanisms in ns-3**.

Measurement results are verified using:
- comparison with PCAP traces,
- consistency with IEEE 802.11 timing rules and protocol behaviour,
- cross-checking between independent PHY- and MAC-layer statistics.

The scenarios are intentionally kept simple and reproducible to make protocol behaviour
and measurement outcomes easy to interpret.

---

## Repository contents

- Source files for all simulation scenarios (`*.cc`)
- Final PDF of the associated engineering thesis (`Dziunikowska_Analysis_of_Data_Link_Layer_Statistics_in_IEEE_802.11.pdf`)
- `README.md`

---

## Running the simulations

All scenarios are executed using the standard `ns3 run` interface. Parameters are passed
via the command line to enable repeatable experiments without modifying source code.

Example command:
```bash
./ns3 run "scratch/wifi-stats --nWifi=1 --maxPackets=1 --interval=1 --simulationTime=2"
