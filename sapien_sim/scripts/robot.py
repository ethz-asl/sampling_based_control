import sapien.core as sapien


class Robot:
    def __init__(self, robot):
        self.robot: sapien.Articulation = robot
        self.all_links = []
        self.all_links_ids = []
        self.all_links_names = []
        self.movable_links = []
        self.movable_links_ids = []
        self.movable_links_names = []
        self.fill_data()

    def fill_data(self):
        self.all_links = [l for l in self.robot.get_links()]
        self.all_links_ids = [l.get_id() for l in self.robot.get_links()]
        self.all_links_names = [l.get_name() for l in self.robot.get_links()]
        for j in self.robot.get_joints():
            if j.get_dof() == 1:
                self.movable_links.append(j.get_child_link())
                self.movable_links_ids.append(j.get_child_link().get_id())
                self.movable_links_names.append(j.get_child_link().get_name())

    def get_link(self, name):
        if name not in self.all_links_names:
            raise NameError(f'No index named {name}')
        else:
            return self.all_links[self.all_links_names.index(name)]

    def set_pd_gains(self, p: list, d: list):
        active_joints = [
            joint for joint in self.robot.get_joints() if joint.get_dof() > 0
        ]
        num_joints = len(active_joints)
        if len(p) != num_joints and len(d) != num_joints:
            print(
                f'PD gains must have the same size as the number of joints ({num_joints})'
            )

        # set joint property
        for idx, joint in enumerate(active_joints):
            joint.set_drive_property(stiffness=p[idx], damping=d[idx])

    def compensate_gravity_and_coriolis(self):
        qf = self.robot.compute_passive_force(gravity=True,
                                              coriolis_and_centrifugal=True,
                                              external=True)
        self.robot.set_qf(qf)

    def __str__(self):
        msg = f'All links ids: {self.all_links_ids}\n' \
              f'All links names: {self.all_links_names}\n' \
              f'All movable links ids: {self.movable_links_ids}\n' \
              f'All movable links names: {self.movable_links_names}'
        return msg


def load_robot(scene, robot_name, urdf_filename, verbose=False):
    loader = scene.create_urdf_loader()
    loader.fix_root_link = True
    robot = loader.load(urdf_filename)
    robot.set_root_pose(sapien.Pose([0, 0, 0], [1, 0, 0, 0]))

    return Robot(robot)
