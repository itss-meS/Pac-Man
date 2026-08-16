# python/pacbot/core/motor_model.py
import numpy as np
from ..core.entities import PacBot
from ..core.maze import Maze
from ..utils.config import CFG

try:
    import pacbot_core as core_cpp
    CPP_CTRL = True
except ImportError:
    CPP_CTRL = False

class MotorController:
    def __init__(self, maze: Maze):
        self.maze = maze
        if CPP_CTRL:
            cfg = core_cpp.KinematicConfig()
            cfg.wheel_base = 0.5
            cfg.max_v_lin = CFG.PACBOT_MAX_SPEED
            cfg.max_v_ang = 5.0
            cfg.max_acc_lin = CFG.PACBOT_MAX_ACCEL
            cfg.max_acc_ang = 10.0
            self.ctrl = core_cpp.DiffDriveController(cfg)
        else:
            self.ctrl = None
            # Fallback PID State
            self.int_lin = self.int_ang = 0.0
            self.prev_lin = self.prev_ang = 0.0

    def update(self, bot: PacBot, dt: float) -> Tuple[float, float]:
        if not bot.path:
            if CPP_CTRL: self.ctrl.command_velocity(0, 0)
            return 0.0, 0.0

        # Target: Next waypoint (Lookahead 1-2 cells)
        idx = min(2, len(bot.path) - 1)
        tx, ty = bot.path[idx][0] + 0.5, bot.path[idx][1] + 0.5
        target_pose = core_cpp.Pose(tx, ty) if CPP_CTRL else (tx, ty)

        if CPP_CTRL:
            wheels = self.ctrl.update(dt, target_pose)
            vl, vr = wheels.left, wheels.right
            # Sync True Pose from C++ Controller (Source of Truth for Sim)
            state = self.ctrl.get_state()
            bot.pose.x, bot.pose.y, bot.pose.theta = state.pose.pos.x, state.pose.pos.y, state.pose.theta
            bot.velocity, bot.omega = state.v_lin, state.v_ang
        else:
            vl, vr = self._fallback_pid(bot, target_pose, dt)

        # Collision Handling (Slide) - Only if not using C++ physics integration
        # C++ Kinematics currently doesn't know maze walls. 
        # We do a post-step correction here for Sim Accuracy.
        self._handle_collision(bot)

        # Consume Waypoint
        if bot.path and bot.pose.dist_to_pt((bot.path[0][0]+0.5, bot.path[0][1]+0.5)) < 0.35:
            bot.path.pop(0)

        bot.wheel_vel = (vl, vr)
        return vl, vr

    def _fallback_pid(self, bot: PacBot, target: Tuple[float,float], dt: float):
        dx, dy = target[0] - bot.pose.x, target[1] - bot.pose.y
        dist = np.hypot(dx, dy)
        tgt_ang = np.arctan2(dy, dx)
        ang_err = (tgt_ang - bot.pose.theta + np.pi) % (2*np.pi) - np.pi

        kp_lin, ki_lin, kd_lin = 4.0, 0.1, 0.5
        kp_ang, ki_ang, kd_ang = 6.0, 0.0, 0.3
        
        self.int_lin = np.clip(self.int_lin + dist*dt, -10, 10)
        self.int_ang = np.clip(self.int_ang + ang_err*dt, -5, 5)
        d_lin = (dist - self.prev_lin)/dt
        d_ang = (ang_err - self.prev_ang)/dt
        
        v_cmd = kp_lin*dist + ki_lin*self.int_lin + kd_lin*d_lin
        w_cmd = kp_ang*ang_err + ki_ang*self.int_ang + kd_ang*d_ang
        
        v_cmd = np.clip(v_cmd, -CFG.PACBOT_MAX_SPEED, CFG.PACBOT_MAX_SPEED)
        w_cmd = np.clip(w_cmd, -5.0, 5.0)
        
        # Accel Limit
        dv = v_cmd - bot.velocity
        if abs(dv) > CFG.PACBOT_MAX_ACCEL * dt: v_cmd = bot.velocity + np.sign(dv)*CFG.PACBOT_MAX_ACCEL*dt
        
        bot.velocity, bot.omega = v_cmd, w_cmd
        self.prev_lin, self.prev_ang = dist, ang_err
        
        # Integrate
        bot.pose.theta += w_cmd * dt
        bot.pose.x += v_cmd * np.cos(bot.pose.theta) * dt
        bot.pose.y += v_cmd * np.sin(bot.pose.theta) * dt
        
        L = 0.5
        vl = v_cmd - w_cmd * L / 2
        vr = v_cmd + w_cmd * L / 2
        return vl, vr

    def _handle_collision(self, bot: PacBot):
        # Circular Robot Radius
        r = 0.35
        # Check 4 corners of bounding box
        c, s = np.cos(bot.pose.theta), np.sin(bot.pose.theta)
        corners = [
            (bot.pose.x + r*c - r*s, bot.pose.y + r*s + r*c),
            (bot.pose.x + r*c + r*s, bot.pose.y + r*s - r*c),
            (bot.pose.x - r*c - r*s, bot.pose.y - r*s + r*c),
            (bot.pose.x - r*c + r*s, bot.pose.y - r*s - r*c),
        ]
        push = np.array([0.0, 0.0])
        hit = False
        for cx, cy in corners:
            ix, iy = int(cx), int(cy)
            if self.maze.is_wall(ix, iy):
                hit = True
                wcx, wcy = ix + 0.5, iy + 0.5
                dx, dy = cx - wcx, cy - wcy
                d = np.hypot(dx, dy)
                if d > 1e-6: push += np.array([dx, dy]) / d * (r - d)
        
        if hit:
            bot.pose.x += push[0] * 0.5
            bot.pose.y += push[1] * 0.5
            bot.velocity *= 0.1
            bot.omega *= 0.5
            if CPP_CTRL: self.ctrl.set_state(self.ctrl.get_state()) # Resync C++ state
