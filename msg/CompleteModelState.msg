#Complete Model State message. Involves the original model state, joint states
gazebo_msgs/ModelState model_state #The pose and twist of the model
JointState[] joint_states #The state of the joints
bool change_modelstate #Specifies if you have to change the model state. If set to false only sets the joint states
