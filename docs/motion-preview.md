# Motion preview implementation

The authoritative contract is in Apifox project 8387612, ROCOS API module 7912103. This file describes the implementation, not a maintained OpenAPI copy.

Integration status (2026-11-03): published to Apifox project 8387612, module 7912103, main branch through the authorized official import API. MCP refresh and readback verified the preview operation, all three new schemas, unchanged existing operations/security definitions and API version 3.0.0. The MCP connection provides read/refresh capabilities.

`POST /api/robot/motion/preview` computes a trajectory without entering the robot state machine or sending hardware commands. It does not require enable state or control ownership. `start_joints` is explicit so a client can chain virtual moves without changing telemetry. The supported types are MoveJ, MoveJ_IK, MoveL and MoveL_FK. Cartesian targets are absolute base-to-flange poses; tool/reference transforms are resolved by the client before requesting a preview.

Each request uses a separate planner and a cloned model with independent solvers. One preview per Robot may run at a time. The existing control loop is not locked while planning. Motion preview and PositionController share finite-value, dimension, joint-position and command-step velocity checks. All control-cycle points are checked before display downsampling. IK/FK failures, unfinished endpoints, oversized trajectories and computation-budget failures return an error without a partial result.

Planning allows up to 5 seconds of computation, 120 seconds of trajectory and 120000 steps. The response retains the start and endpoint and at most max_points samples (default 300, range 2–2000). A stationary motion returns one point and duration zero. Display samples are not an executable control-cycle trajectory. Collision, force-control, hardware readiness and other live execution checks are not included.

No plan ID is retained. Real execution still uses the existing motion endpoints and starts from actual feedback at execution time. A successful preview does not bypass any live controller checks or guarantee that re-planning reproduces the same path.

Validation is offline: motion_preview_test runs the real planners and a loopback HTTP server without constructing Robot/Hardware; position_controller_test uses fake hardware. Do not use a real controller as a preview test fixture.
