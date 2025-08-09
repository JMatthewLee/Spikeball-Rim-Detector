<h1>Spikeball Rim Detector</h1>
<p>
  <b>Arduino-based impact detection system for Spikeball games with instant LED feedback</b>
</p>
<b>What It Does:</b>

Detects rim hits vs net shots using MPU6500 accelerometer analysis
Instant LED strip feedback with flash sequences
Smart detection logic: lateral acceleration = rim hit, vertical = net shot
Fast response: 2ms sampling with 400ms debounce protection

<b>Technical Specs:</b>

<code>MPU6500</code> 6-axis sensor (±8g range)
<code>Threshold Acceleration</code> for both rim and net detection
<code>LED strip control</code> via MOSFET switching

<hr>
<div align="center">
  <strong>Status:</strong> Working prototype | 
  <strong>Components:</strong> Arduino Nano + MPU6500 + LED strip
  <br><br>
  <em>Circuit diagrams and demo videos coming soon</em>
</div>
