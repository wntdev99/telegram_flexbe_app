#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from telegram_flexbe_states.change_robot_state import ChangeRobotState
from telegram_flexbe_states.check_navigation import CheckNavigationState
from telegram_flexbe_states.check_patrol_state import CheckPotralState
from telegram_flexbe_states.check_person import CheckPerson
from telegram_flexbe_states.clear_file import ClearFileState
from telegram_flexbe_states.communicate_telegram_node_state import CommunicateTelegramNodeState
from telegram_flexbe_states.get_goal_pose import GetGoalPose
from telegram_flexbe_states.initialization_kobuki import InitializationKobuki
from telegram_flexbe_states.navigation_kobuki import NavigationKobuki
from telegram_flexbe_states.potral_door import PointCloudTestState
from telegram_flexbe_states.send_pictures import SendPictures
from telegram_flexbe_states.send_results import SendResults
from telegram_flexbe_states.separate_command_message_state import SeparateCommandMessageState
from telegram_flexbe_states.take_a_picture import TakePicture
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Apr 04 2024
@author: Jeongmin Choi
'''
class telegram_flexbe_appSM(Behavior):
	'''
	This is a flexbe application that executes commands to the Kobuki robot via messages received from Telegram. The FlexBe application can perform four commands: Call, Charge, and Patrol.
	'''


	def __init__(self):
		super(telegram_flexbe_appSM, self).__init__()
		self.name = 'telegram_flexbe_app'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		topic_initialpose = "/initialpose"
		service_state = "/robot_state_service"
		path_check_image = "/home/develop/katkin_ws/src/telegram_behaviors/telegram_flexbe_states/src/telegram_flexbe_states/data/image/"
		path_patrol_image = "/home/develop/katkin_ws/src/telegram_behaviors/telegram_flexbe_states/src/telegram_flexbe_states/data/image/"
		path_yolo_result = "/home/develop/.ros/runs/detect"
		path_yolo_model = "/home/develop/katkin_ws/src/telegram_behaviors/telegram_flexbe_states/src/telegram_flexbe_states/data/yolov8x.pt"
		service_message = '/message_service'
		path_excel_data = '/home/develop/katkin_ws/src/telegram_behaviors/telegram_flexbe_states/src/telegram_flexbe_states/data/pose_data.xlsx'
		action_move_base = "/move_base_flex/move_base"
		topic_pointcloud2 = "/camera/depth/color/points"
		token = 'my token'
		chat_id = 'chat id'
		message_door_open = "문이 열려있습니다."
		message_door_close = "문이 닫혀있습니다."
		topic_camera = "/usb_cam/camera"
		# x:30 y:365, x:20 y:1230
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:85 y:74
			OperatableStateMachine.add('initialization_pose',
										InitializationKobuki(topic=topic_initialpose),
										transitions={'OK': 'robot_is_ready'},
										autonomy={'OK': Autonomy.Off})

			# x:1485 y:159
			OperatableStateMachine.add('check_door_state',
										CheckPotralState(image_path=path_check_image),
										transitions={'navigation': 'get_door_pose', 'finish': 'check_person_pose'},
										autonomy={'navigation': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'pose_name': 'pose_name'})

			# x:1435 y:909
			OperatableStateMachine.add('check_person_count',
										CheckPerson(image_path=path_check_image, result_path=path_yolo_result, model_path=path_yolo_model),
										transitions={'results': 'send_person_count_result', 'fail': 'failed'},
										autonomy={'results': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'pose_name': 'pose_name', 'results': 'results'})

			# x:1175 y:665
			OperatableStateMachine.add('check_person_pose',
										CheckNavigationState(image_path=path_check_image),
										transitions={'navigation': 'get_person_pose', 'finish': 'clear_saved_files'},
										autonomy={'navigation': Autonomy.Off, 'finish': Autonomy.Off},
										remapping={'pose_name': 'pose_name'})

			# x:85 y:224
			OperatableStateMachine.add('clear_saved_files',
										ClearFileState(image_path=path_check_image, result_path=path_patrol_image),
										transitions={'next': 'robot_is_ready'},
										autonomy={'next': Autonomy.Off})

			# x:611 y:74
			OperatableStateMachine.add('communication_telegram',
										CommunicateTelegramNodeState(message_service=service_message),
										transitions={'behavior': 'robot_is_busy', 'wait': 'communication_telegram'},
										autonomy={'behavior': Autonomy.Off, 'wait': Autonomy.Off},
										remapping={'state': 'state', 'get_message': 'get_message'})

			# x:835 y:325
			OperatableStateMachine.add('get_call_pose',
										GetGoalPose(excel_path=path_excel_data),
										transitions={'navigation': 'navigation_to_call'},
										autonomy={'navigation': Autonomy.Off},
										remapping={'pose_name': 'pose_name', 'goal_pose': 'goal_pose'})

			# x:1075 y:386
			OperatableStateMachine.add('get_charge_pose',
										GetGoalPose(excel_path=path_excel_data),
										transitions={'navigation': 'navigation_to_charge'},
										autonomy={'navigation': Autonomy.Off},
										remapping={'pose_name': 'pose_name', 'goal_pose': 'goal_pose'})

			# x:1485 y:324
			OperatableStateMachine.add('get_door_pose',
										GetGoalPose(excel_path=path_excel_data),
										transitions={'navigation': 'navigation_to_door'},
										autonomy={'navigation': Autonomy.Off},
										remapping={'pose_name': 'pose_name', 'goal_pose': 'goal_pose'})

			# x:1185 y:824
			OperatableStateMachine.add('get_person_pose',
										GetGoalPose(excel_path=path_excel_data),
										transitions={'navigation': 'navigation_to_person'},
										autonomy={'navigation': Autonomy.Off},
										remapping={'pose_name': 'pose_name', 'goal_pose': 'goal_pose'})

			# x:535 y:324
			OperatableStateMachine.add('navigation_to_call',
										NavigationKobuki(move_base_action=action_move_base),
										transitions={'results': 'send_call_result'},
										autonomy={'results': Autonomy.Off},
										remapping={'goal_pose': 'goal_pose', 'pose_name': 'pose_name', 'results': 'results'})

			# x:750 y:453
			OperatableStateMachine.add('navigation_to_charge',
										NavigationKobuki(move_base_action=action_move_base),
										transitions={'results': 'send_charge_result'},
										autonomy={'results': Autonomy.Off},
										remapping={'goal_pose': 'goal_pose', 'pose_name': 'pose_name', 'results': 'results'})

			# x:1485 y:422
			OperatableStateMachine.add('navigation_to_door',
										NavigationKobuki(move_base_action=action_move_base),
										transitions={'results': 'send_door_navi_result'},
										autonomy={'results': Autonomy.Off},
										remapping={'goal_pose': 'goal_pose', 'pose_name': 'pose_name', 'results': 'results'})

			# x:1185 y:924
			OperatableStateMachine.add('navigation_to_person',
										NavigationKobuki(move_base_action=action_move_base),
										transitions={'results': 'send_person_navi_result'},
										autonomy={'results': Autonomy.Off},
										remapping={'goal_pose': 'goal_pose', 'pose_name': 'pose_name', 'results': 'results'})

			# x:985 y:67
			OperatableStateMachine.add('robot_is_busy',
										ChangeRobotState(service=service_state),
										transitions={'OK': 'understanding_command'},
										autonomy={'OK': Autonomy.Off},
										remapping={'state': 'state'})

			# x:335 y:74
			OperatableStateMachine.add('robot_is_ready',
										ChangeRobotState(service=service_state),
										transitions={'OK': 'communication_telegram'},
										autonomy={'OK': Autonomy.Off},
										remapping={'state': 'state'})

			# x:285 y:324
			OperatableStateMachine.add('send_call_result',
										SendResults(token=token, chat_id=chat_id),
										transitions={'next': 'clear_saved_files', 'fail': 'failed'},
										autonomy={'next': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'results': 'results'})

			# x:452 y:441
			OperatableStateMachine.add('send_charge_result',
										SendResults(token=token, chat_id=chat_id),
										transitions={'next': 'clear_saved_files', 'fail': 'failed'},
										autonomy={'next': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'results': 'results'})

			# x:1485 y:509
			OperatableStateMachine.add('send_door_navi_result',
										SendResults(token=token, chat_id=chat_id),
										transitions={'next': 'take_picture', 'fail': 'failed'},
										autonomy={'next': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'results': 'results'})

			# x:1679 y:267
			OperatableStateMachine.add('send_door_open_result',
										SendResults(token=token, chat_id=chat_id),
										transitions={'next': 'send_door_picture', 'fail': 'failed'},
										autonomy={'next': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'results': 'results'})

			# x:1707 y:167
			OperatableStateMachine.add('send_door_picture',
										SendPictures(token=token, chat_id=chat_id, image_path=path_check_image, result_path=path_patrol_image),
										transitions={'next': 'check_door_state'},
										autonomy={'next': Autonomy.Off},
										remapping={'pose_name': 'pose_name'})

			# x:1422 y:759
			OperatableStateMachine.add('send_person_count_result',
										SendResults(token=token, chat_id=chat_id),
										transitions={'next': 'send_person_picture', 'fail': 'failed'},
										autonomy={'next': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'results': 'results'})

			# x:1185 y:1059
			OperatableStateMachine.add('send_person_navi_result',
										SendResults(token=token, chat_id=chat_id),
										transitions={'next': 'take_person_picture', 'fail': 'failed'},
										autonomy={'next': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'results': 'results'})

			# x:1432 y:655
			OperatableStateMachine.add('send_person_picture',
										SendPictures(token=token, chat_id=chat_id, image_path=path_patrol_image, result_path=path_yolo_result),
										transitions={'next': 'check_person_pose'},
										autonomy={'next': Autonomy.Off},
										remapping={'pose_name': 'pose_name'})

			# x:1435 y:1067
			OperatableStateMachine.add('take_person_picture',
										TakePicture(camera_topic=topic_camera, image_path=path_check_image, blocking=True, clear=False),
										transitions={'complete': 'check_person_count'},
										autonomy={'complete': Autonomy.Off},
										remapping={'pose_name': 'pose_name'})

			# x:1685 y:467
			OperatableStateMachine.add('take_picture',
										TakePicture(camera_topic=topic_camera, image_path=path_check_image, blocking=True, clear=False),
										transitions={'complete': 'check_door_open'},
										autonomy={'complete': Autonomy.Off},
										remapping={'pose_name': 'pose_name'})

			# x:1061 y:174
			OperatableStateMachine.add('understanding_command',
										SeparateCommandMessageState(token=token, chat_id=chat_id, excel_path=path_excel_data),
										transitions={'call': 'get_call_pose', 'patrol': 'check_door_state', 'charge': 'get_charge_pose', 'fail': 'clear_saved_files'},
										autonomy={'call': Autonomy.Off, 'patrol': Autonomy.Off, 'charge': Autonomy.Off, 'fail': Autonomy.Off},
										remapping={'get_message': 'get_message', 'pose_name': 'pose_name'})

			# x:1688 y:354
			OperatableStateMachine.add('check_door_open',
										PointCloudTestState(pointcloud2_topic=topic_pointcloud2, message_door_open=message_door_open, message_door_close=message_door_close, blocking=True, clear=False),
										transitions={'complete': 'send_door_open_result'},
										autonomy={'complete': Autonomy.Off},
										remapping={'pose_name': 'pose_name', 'results': 'results'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
