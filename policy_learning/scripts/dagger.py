#!/usr/bin/env python3

import os
import datetime
import torch
import roslaunch
import rospy

from learning import PolicyLearner
from dataset import StateActionDataset


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
    dataset_name = "PandaR10_sStart_sGoal_0504163457"
    dir_path = task_path + "/../data/" + dataset_name
    dir_path = os.path.join(task_path, os.pardir, 'data', dataset_name)
    initial_dataset = StateActionDataset(root_dir=dir_path)
    # define a new folder where all dagger saving and loading takes place
    dagger_path = os.path.join(dir_path,
        datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')+'_dagger')
    dagger_models_path = os.path.join(dagger_path, 'policies')
    dagger_datasets_path = os.path.join(dagger_path, 'datasets')
    if (not os.path.isdir(dagger_models_path)):
        os.makedirs(dagger_models_path)
    if (not os.path.isdir(dagger_datasets_path)):
        os.makedirs(dagger_datasets_path)
    model_path = os.path.join(dagger_models_path, 'iter_0')
    # fix a random seed
    torch.manual_seed(1)
    idx=1
    sample = initial_dataset[idx]

    # set device for training
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    # train first policy
    learner = PolicyLearner(dir_path, device)
    learner.train()
    # save first policy, will be loaded from the ros node
    learner.save_model(sample['state'], model_path)


    ##############################
    # loop
    iterations = 10
    for i in range(iterations):
        ## launch a dagger node -> this also saves a new dataset file
        # set name for this dataset file
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)


        dataset_path = os.path.join(dagger_datasets_path, f'iter_{i+1}.hdf5')
        model_path = os.path.join(dagger_models_path, f'iter_{i}.pt')

        print('Dataset path: ', dataset_path)
        print('Model path: ', model_path)

        cli_args = ['mppi_panda', 'panda_dagger.launch',
                        f'learner_output_path:={dataset_path}',
                        f'torchscript_model_path:={model_path}']
        print(cli_args)
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
        print(roslaunch_file)
        parent = roslaunch.parent.ROSLaunchParent(uuid,
                                            [(roslaunch_file, cli_args[2:])],
                                            force_required=True,
                                            timeout=60)

        parent.start()

        rospy.sleep(10)
        parent.shutdown()

        ## load the newly created datafile
        ## some kind of function which joins the datasets (and also saves the
        ## larger dataset and deletes the other one?)
        learner.update_dataset()

        ## train the classifier using data from this new dataset

        idx=1
        sample = dataset[idx]

        # train update policy
        ## might need a new method on PolicyLerning to update the dataset or just
        ## intialise a new one
        learner.train()
        # save update policy, will be loaded from the ros node, make sure naming is consistent
        save_model_path = os.path.join(dagger_models_path, f'iter_{i+1}')
        learner.save_model(sample['state'], save_model_path)
