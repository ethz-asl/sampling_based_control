import sapien.core as sapien
from sapien.core import Pose, SceneConfig, OptifuserConfig, VulkanRenderer
import numpy as np
import time
from robot import Robot, load_robot

from PIL import ImageColor, Image
import open3d


class ContactError(Exception):
    pass


class Env(object):
    def __init__(self, show_gui=True, render_rate=20, timestep=1 / 240):
        self.current_step = 0

        self.show_gui = show_gui
        self.render_rate = render_rate
        self.timestep = timestep

        # engine and renderer
        self.engine = sapien.Engine(0, 0.001, 0.005)

        render_config = OptifuserConfig()
        render_config.shadow_map_size = 8192
        render_config.shadow_frustum_size = 10
        render_config.use_shadow = False
        render_config.use_ao = True

        self.renderer = sapien.VulkanRenderer(False)  # True for headless
        self.engine.set_renderer(self.renderer)

        # GUI
        self.window = False
        if show_gui:
            self.renderer_controller = sapien.VulkanController(self.renderer)
            self.renderer_controller.set_free_camera_position(-3.0, 1.0, 3.0)
            self.renderer_controller.set_free_camera_rotation(-0.4, -0.8, 0.0)
            self.renderer_controller.set_default_control(True, True)

        # scene
        scene_config = SceneConfig()
        scene_config.gravity = [0, 0, -9.81]
        scene_config.solver_iterations = 20
        scene_config.enable_pcm = False
        scene_config.sleep_threshold = 0.0

        self.scene = self.engine.create_scene(config=scene_config)
        if show_gui:
            self.renderer_controller.set_current_scene(self.scene)

        self.scene.set_timestep(timestep)

        # add lights
        self.scene.set_ambient_light([0.5, 0.5, 0.5])
        self.scene.set_shadow_light([0, 1, -1], [0.5, 0.5, 0.5])
        self.scene.add_point_light([1, 2, 2], [1, 1, 1])
        self.scene.add_point_light([1, -2, 2], [1, 1, 1])
        self.scene.add_point_light([-1, 0, 1], [1, 1, 1])
        self.scene.add_ground(altitude=-0.1)

        # default Nones
        self.object = None
        self.object_target_joint = None

        # check contact
        self.check_contact = False

        # agents
        self.robots = {}
        self.cameras = {}

    def add_camera(self, camera_name, actor: sapien.ActorBase, pose: Pose):
        """
        Orientation is that x axis of the camera is pointing forward
        :param camera_name:
        :param actor:
        :param pose:
        :return:
        """
        near, far = 0.1, 100
        camera = self.scene.add_mounted_camera(camera_name, actor, pose, 640,
                                               480, np.deg2rad(90),
                                               np.deg2rad(90), near, far)
        self.cameras[camera_name] = camera

    def add_camera_to_robot(self, camera_name, pose, link_name, robot_name):
        if robot_name not in self.robots.keys():
            print(f'No agent named {robot_name}')
            return
        link_actor = self.robots[robot_name].get_link(link_name)
        self.add_camera(camera_name, link_actor, pose)

    def set_controller_camera_pose(self, x, y, z, yaw, pitch, roll):
        self.renderer_controller.set_free_camera_position(x, y, z)
        self.renderer_controller.set_free_camera_rotation(yaw, pitch, roll)
        self.renderer_controller.render()

    def add_robot(self, robot_name, urdf_filename, verbose=True) -> Robot:
        self.robots[robot_name] = load_robot(self.scene,
                                             robot_name,
                                             urdf_filename,
                                             verbose=verbose)
        return self.robots[robot_name]

    def get_rgba(self, camera_name):
        rgba = self.cameras[camera_name].get_color_rgba()
        rgba = (rgba * 255).clip(0, 255).astype("uint8")
        rgba = Image.fromarray(rgba)

    def get_depth(self, camera_name, pcl=False):
        depth = self.cameras[camera_name].get_depth()
        points = self.cameras[camera_name].get_position_rgba()[depth != 1]
        points[..., 3] = 1
        points[..., [0, 1, 2]] = points[..., [2, 0, 1]]
        points[..., [0, 1]] *= -1
        camera_pose = self.cameras[camera_name].get_pose(
        ).to_transformation_matrix()
        points = (camera_pose @ points.T).T[:, :3]

        if pcl:
            y, x = np.where(depth < 1)
            cloud = open3d.geometry.PointCloud()
            cloud.points = open3d.utility.Vector3dVector(points)
            obj_segmentation = self.cameras[camera_name].get_obj_segmentation(
            )[y, x]
            color_map = (np.array([
                ImageColor.getrgb(color)
                for color in ImageColor.colormap.keys()
            ]) / 255)
            cloud.colors = open3d.utility.Vector3dVector(
                color_map[obj_segmentation])

            # open3d.io.write_point_cloud("cloud.pcd", cloud)
            open3d.visualization.draw_geometries([cloud])

    def get_object_qpos(self):
        return self.object.get_qpos()

    def get_target_part_qpos(self):
        qpos = self.object.get_qpos()
        return float(qpos[self.target_object_part_joint_id])

    def get_target_part_pose(self):
        return self.target_object_part_actor_link.get_pose()

    def start_checking_contact(self, robot_hand_actor_id,
                               robot_gripper_actor_ids, strict):
        self.check_contact = True
        self.check_contact_strict = strict
        self.first_timestep_check_contact = True
        self.robot_hand_actor_id = robot_hand_actor_id
        self.robot_gripper_actor_ids = robot_gripper_actor_ids

    def get_material(self, static_friction, dynamic_friction, restitution):
        return self.engine.create_physical_material(static_friction,
                                                    dynamic_friction,
                                                    restitution)

    def render(self):
        if self.show_gui and (not self.window):
            self.window = True
            self.renderer_controller.render()
        self.scene.update_render()
        if self.show_gui and (self.current_step % self.render_rate == 0):
            self.renderer_controller.render()

    def step(self):
        self.current_step += 1
        self.scene.step()
        # if self.check_contact:
        #     if not self.check_contact_is_valid():
        #         raise ContactError()

    # check the first contact: only gripper links can touch the target object part link
    def check_contact_is_valid(self):
        self.contacts = self.scene.get_contacts()
        contact = False
        valid = False
        for c in self.contacts:
            aid1 = c.actor1.get_id()
            aid2 = c.actor2.get_id()
            has_impulse = False
            for p in c.points:
                if abs(p.impulse @ p.impulse) > 1e-4:
                    has_impulse = True
                    break
            if has_impulse:
                if (aid1 in self.robot_gripper_actor_ids and aid2 == self.target_object_part_actor_id) or \
                        (aid2 in self.robot_gripper_actor_ids and aid1 == self.target_object_part_actor_id):
                    contact, valid = True, True
                if (aid1 in self.robot_gripper_actor_ids and aid2 in self.non_target_object_part_actor_id) or \
                        (aid2 in self.robot_gripper_actor_ids and aid1 in self.non_target_object_part_actor_id):
                    if self.check_contact_strict:
                        return False
                    else:
                        contact, valid = True, True
                if (aid1 == self.robot_hand_actor_id
                        or aid2 == self.robot_hand_actor_id):
                    if self.check_contact_strict:
                        return False
                    else:
                        contact, valid = True, True
                # starting pose should have no collision at all
                if (aid1 in self.robot_gripper_actor_ids
                        or aid1 == self.robot_hand_actor_id
                        or aid2 in self.robot_gripper_actor_ids
                        or aid2 == self.robot_hand_actor_id
                    ) and self.first_timestep_check_contact:
                    return False

        self.first_timestep_check_contact = False
        if contact and valid:
            self.check_contact = False
        return True

    def close_render(self):
        if self.window:
            self.renderer_controller.hide_window()
        self.window = False

    def wait_to_start(self):
        print('press q to start\n')
        while not self.renderer_controller.should_quit:
            self.scene.update_render()
            if self.show_gui:
                self.renderer_controller.render()

    def should_close(self):
        return self.renderer_controller.is_closed

    def close(self):
        if self.show_gui:
            self.renderer_controller.set_current_scene(None)
        self.scene = None


if __name__ == "__main__":
    env = Env()
    target = [
        0.0, 0.0, 0.0, 0.0, -0.52, 0.0, -1.785, 0.0, 1.10, 0.69, 0.0, 0.0
    ]
    royal_panda = env.add_robot(robot_name='royal_panda',
                                urdf_filename='../urdf/royal_panda.urdf',
                                verbose=True)
    royal_panda.set_pd_gains([10] * 12, [10] * 12)
    royal_panda.robot.set_drive_target(target)

    shelf = env.add_robot(robot_name='shelf',
                          urdf_filename='../urdf/shelf.urdf',
                          verbose=True)

    # Attach camera to panda hand
    pose = Pose([0.0, 0.0, 0.05])
    env.add_camera_to_robot('camera_test',
                            robot_name='royal_panda',
                            link_name='panda_hand',
                            pose=pose)

    while not env.should_close():
        royal_panda.compensate_gravity_and_coriolis()
        royal_panda.robot.set_drive_target(target)
        env.step()
        env.render()
        env.get_depth('camera_test', pcl=False)
