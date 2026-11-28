# Case study: a defensive deadline and independent clocks

During AI-assisted development of the OBot camera controls, a short request
deadline and sequence check were added to prevent delayed movement commands
from restarting motion after cancellation. The deadline was calculated on the
laptop and checked against the robot's clock. This created an undocumented
requirement for the two independent computers' clocks to agree, despite the
robot operating on an isolated wireless network.

Automated tests used matching clocks and passed. Physical testing exposed the
problem: the viewer could connect to the motion service but movement was
rejected. The original message combined several possible rejection causes.
A read-only diagnostic with a new client and measured clock offset reproduced
the future-deadline rejection.

The revised design uses robot-issued, short-lived, single-use movement permits
and robot-local monotonic time, plus sequence and cancellation barriers. STOP
does not depend on the movement request's freshness check. Independent-clock
and actual service tests were added using mocked motion hardware.

**Lesson:** Defensive programming must fit the actual operating environment.
Passing automated tests does not establish operational correctness; test
cross-computer assumptions explicitly and include physical system testing.
