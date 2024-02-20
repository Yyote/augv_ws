drone_msgs/FlightControlMode mode # Режим, в котором будет проходиться данный сегмент
drone_msgs/GoalTrajectory trajectory # Траектория, по которой будет выполняться данная миссия
# uint8 priority # Приоритет

uint8 completion_condition # Условие, по которому текущий сегмент считается выполненным

uint8 CUSTOM_CONDITION_TOPIC = 0 # Пользовательское условие, статус которого публикуется сообщением std_msgs/Bool в топик custom_condition_topic
uint8 LAST_POINT_ARRIVAL = 1 # Достижение последней точки в траектории

std_msgs/String custom_condition_topic # Название топика, в который публикуется статус пользовательского условия