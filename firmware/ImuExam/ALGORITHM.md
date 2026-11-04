# Algorithm, assumptions, and attribution

The code in this folder is original project code. It implements the standard
quaternion kinematics and gravity-vector proportional feedback equations of
Mahony's nonlinear complementary attitude filter. Reference publication:
R. Mahony, T. Hamel, J.-M. Pflimlin, "Nonlinear Complementary Filters on the
Special Orthogonal Group", IEEE Transactions on Automatic Control, 2008,
https://doi.org/10.1109/TAC.2008.923738 . Implementation background and reference
algorithms: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/ . No source
from that site or another implementation is copied into this project.

Quaternion order is wxyz; q actively rotates sensor/body vectors to a right
handed world frame with Z up. Roll/pitch initialize from stationary gravity;
yaw starts at zero. Proportional gain Kp=0.5, integral gain Ki=0. The startup
calibration independently estimates gyro bias. No magnetometer is present:
yaw is relative and can drift. Acceleration magnitude outside 0.8..1.2g is
excluded from tilt correction, but this cannot reject all dynamic acceleration.

The software low pass is y += alpha*(x-y), alpha=1-exp(-2*pi*20*dt), nominally
alpha=0.466512 at 200 Hz. Hardware DLPF=44 Hz applies equally to both raw and
processed streams. Raw ADC values are never overwritten with calibrated data.

Startup calibration requires an operator-held stationary sensor and at least
3 seconds/600 stable samples. Acceleration norm must remain within 0.5..1.5g;
axis changes and variance reject movement. Gyro mean is estimated rather than
rejected based on its DC magnitude. Constant rotation or constant external
acceleration cannot be ruled out by this stationarity test: the operator must
actually leave the module still.

The measured device produced approximately 1.19g while stationary. As a limited
single-pose normalization, accel_norm_scale = g / norm(mean raw acceleration)
multiplies only the processed acceleration before its low pass. Raw counts and
their nominal SI conversion remain untouched. The exact factor and method are
saved in META. This is an isotropic norm correction, not a six-face offset and
per-axis scale calibration. It cannot identify sensor offset, anisotropic gain,
or mounting tilt; other orientations can still have substantial errors. Its
agreement with g in the calibration pose is enforced by construction and is
not independent evidence of accuracy. Measurement-error tests must use a new
post-calibration dataset and report both raw and processed errors honestly.

Position uses R(q)*accel-[0,0,9.80665], then trapezoidal integration. The
zero-velocity heuristic requires gyro norm below 2 deg/s, acceleration
norm within 0.04g, and world linear-acceleration norm below 0.35 m/s2 for
200 ms. It is only suitable for short rest-to-rest
demonstrations; constant-speed translation is indistinguishable from rest.
It resets velocity but never automatically resets position. No claim of
long-term accurate position or absolute heading is made. ZERO resets only
position and velocity. Gaps over 20 ms are flagged and not integrated across.

Adafruit MPU6050, BusIO, and Unified Sensor are installed external dependencies
with their original upstream licenses. Adafruit configures the sensor; this
project performs checked I2C reads because getEvent() in the installed version
does not propagate bus errors.

