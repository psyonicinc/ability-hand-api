# TCP-Enabled Hand Validation

This modified version of the hand validation script uses TCP communication instead of terminal input for controlling the validation tests.

## Features

- **TCP Server**: Runs on port 12345 (configurable)
- **JSON Data Transmission**: All test data is sent over TCP in JSON format
- **Command-Based Control**: Use simple string commands to control tests
- **Real-time Feedback**: Receive status updates and data during tests
- **Local Plotting**: Plots are still generated locally for immediate visualization

## Usage

### Starting the Server

1. Run the validation script:
   ```bash
   python validate_hand_TCP.py
   ```

2. The server will start and wait for a TCP connection on port 12345

### TCP Client

Use the provided test client or create your own TCP client:

```bash
python tcp_client_test.py
```

## Available Commands

### Test Control
- `start` - Start the current test
- `skip` - Skip the current test
- `next` - Move to the next test (after current test completes)

### User Input
- `y` - Yes (for clinical hand question)
- `n` - No (for clinical hand question)
- `ready` - Ready for next step (during torque tests)

## Test Flow

1. **Grip Validation** - Tests keygrip and hand wave functionality
2. **FSR Validation** - Tests force-sensitive resistors
3. **Velocity Validation** - Tests velocity control mode
4. **Position Validation** - Tests position control mode
5. **Torque Validation** - Tests torque/current control mode

## Data Format

All test data is sent as JSON with the following structure:

```json
{
  "test_type": "velocity_validation",
  "target_velocity": 10,
  "positions": [[...], [...], ...],
  "currents": [[...], [...], ...],
  "velocities": [[...], [...], ...],
  "target_velocities": [[...], [...], ...],
  "timestamps": [...],
  "time_string": "Total Time: 1.23 Target Time: 1.20"
}
```

## Example Client Interaction

```
Server: Validation tests ready. Send 'start' to begin or 'skip' to skip a test.
Client: start
Server: Starting grip validation test
Client: start
Server: Press ready to test keygrip
Client: ready
Server: Move thumb back and forth to test
Client: next
Server: Starting FSR validation test
Client: start
Server: IS THIS A CLINICAL HAND? [y/n]
Client: n
Server: Do not touch index, press ready to continue then press on FSRs
Client: ready
...
```

## Error Handling

- Invalid commands receive "invalid" response
- Valid commands receive "ack" acknowledgment
- Connection errors are logged to console
- Tests can be skipped at any time

## Configuration

- Change `TCP_PORT` in the script to use a different port
- Modify the `TCPValidationServer` class for custom behavior
- Adjust data format in the `send_data()` calls

## Security Notes

- No authentication is implemented
- Use firewall rules to restrict access if needed
- Consider implementing SSL/TLS for production use 