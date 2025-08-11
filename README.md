ROS2 CI Task2
This package is for the CI of ROS2 Fastbot .

To use this:-

git clone https://github.com/robkwan/ros2_ci.git under the local ~/ros2_ws/src folder.

Assuming the Installation of Jenkins ver 2.463 or newer is done properly already
Create a new item from Dashboard, e.g. item name: Robert_ROS2_CI with type select "Freestyle project" and click "OK".

Put some description like "Robert's ROS2 CI Task" and then "Save" and "Build Now" to check if 'Console Output' looks correct.

Go to the project page for the project and click on Configure.

Click on Build, then click on the Add build step dropdown and select Execute Shell since we are on Linux.

Paste the following bash code inside the Command text area:

#!/bin/bash cd /home/user/ros2_ws/src sudo usermod -aG docker $USER & #newgrp docker sudo chmod 666 /var/run/docker.sock & docker ps

and then click on "Add build step" to paste the following bash code into the second command text area:

#!/bin/bash cd /home/user/ros2_ws/src echo "Will check if we need to clone or just pull" if [ ! -d "ros1_ci" ]; then git clone https://github.com/robkwan/ros2_ci.git else cd ros1_c2 git pull origin main fi

and then click again on "Add build step" to paste the following bash code into the third command text area:

cd ~/ros2_ws/src/ros2_ci docker build -t fastbot-ros2-ci . docker-compose up

Afterwards, click "Save" and then "Build now" and check "Console Output" to see if everything is ok.

To add autobuild trigger: Click on Project's "Configure" ==> "Build Triggers"

Set a checkmark on "Poll SCM" Write * * * * * inside the Schedule textbox. if you click on the question mark, you will see that means this will poll our GitHub repository every minute. Then click on Apply and Save.

To add the git repo for polling changes: Click on Project's "Configure" ==> "Source Code Management"

Set up the repository URL to https://github.com/robkwan/ros2_ci.git & Set up the branch to build to: main Then click on Apply and Save.

To test for the changes, simply touch a dummy .txt file in ~/ros2_ws/src/ros2_ci folder and commit it to the git repo. Then auto-build and test should be triggered in the Jenkins for verification.