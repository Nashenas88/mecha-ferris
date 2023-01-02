extends Camera

export(float) var movement_speed = 10.0

func _ready():
	pass

func _physics_process(_delta):
	var directional_input = Vector3.ZERO
	directional_input.x = Input.get_action_strength("ui_right") - Input.get_action_strength("ui_left")
	directional_input.y = Input.get_action_strength("ui_up") - Input.get_action_strength("ui_down")
	directional_input.z = Input.get_action_strength("ui_backward") - Input.get_action_strength("ui_forward")
	translate(directional_input*movement_speed)
	var rotational_input = Vector3.ZERO
	rotational_input.y = Input.get_action_strength("ui_turn_left") - Input.get_action_strength("ui_turn_right")
	rotate_y(rotational_input.y / 100)
	rotational_input.z = Input.get_action_strength("ui_look_up") - Input.get_action_strength("ui_look_down")
	rotate_object_local(Vector3(1, 0, 0), rotational_input.z / 100)
