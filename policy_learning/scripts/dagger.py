import roslaunch
import rospy

from leaning import PolicyLearner


if __name__ == "__main__":

    ## Pseudo algorithm structure  (Ross 2011)
    # load first dataset
    # train first policy pi_0
    # --> done here
    # loop
    #   sample trajectories from pi_1 (potentially use mixing with expert)
    #   --> done in ros node
    #   lable visited states with expert
    #   --> done in ros node
    #   # issue: interface is such that expert is loaded into controller --> potential solution: disable via yaml when launching with roslaunch (learning_ratio)
    #   augment data set
    #   # issue: need to augment dataset from cpp -> no, save data file as currently
    #   # possible, then modify datasets in python
    #   --> done in ros node
    #   train classifier
    #   --> done here

    #######################################
    # load first dataset and train first policy

    task_path = os.path.dirname(os.path.realpath(__file__))
    dataset_name = "Horizon_4_Train_0419140234"
    dir_path = task_path + "/../data/" + dataset_name
    initial_dataset = StateActionDataset(root_dir=dir_path)
    # fix a random seed
    torch.manual_seed(1)
    idx=1
    sample = dataset[idx]

    # set device for training
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    # train first policy
    learner = PolicyLearner(dir_path, device)
    learner.train()
    # save first policy, will be loaded from the ros node
    learner.save_model(sample['state'], dataset_name)


    ##############################
    # loop
    iterations = 10
    for i in range(iterations):
        ## launch a dagger node -> this also saves a new dataset file
        # set name for this dataset file
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        cli_args1 = ['mppi_panda', 'panda_dagger.launch']
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
        parent = rosluanch.parent.ROSLaunchParent(uuid, roslaunch_file)

        parent.start()

        rospy.sleep(10)
        parent.shutdown()

        ## load the newly created datafile
        ## some kind of function which joins the datasets (and also saves the
        ## larger dataset and deletes the other one?)

        ## train the classifier using data from this new dataset
        dataset_name = "aggregated_dataset"
        dir_path = task_path + "/../data/" + dataset_name
        aggregated_dataset = StateActionDataset(root_dir=dir_path)

        idx=1
        sample = dataset[idx]

        # train update policy
        ## might need a new method on PolicyLerning to update the dataset or just
        ## intialise a new one
        learner.train()
        # save update policy, will be loaded from the ros node, make sure naming is consistent
        learner.save_model(sample['state'], dataset_name)
