# CloneRobotics-CodingTask
Robotics Software Engineer: Take home assessment

## Design Decisions and Assumptions

### 1. UART Communication Protocol
- **Specification**: The project brief suggested a text-based UART protocol using `$` and `\n` as delimiters.

- **Implementation**: A custom binary protocol was implemented. Each message is wrapped in a `[HEADER | PAYLOAD | CHECKSUM]` structure for unambiguous parsing and validation.

- **Justification**: This decision was made to better meet the system's core requirements for __error-resistance__ and __performance__. The binary format is more bandwidth-efficient for high-frequency data streams and includes a checksum for data integrity, which is critical for reliable operation. This approach prioritizes robustness and efficiency, key characteristics of a commercial-grade application.