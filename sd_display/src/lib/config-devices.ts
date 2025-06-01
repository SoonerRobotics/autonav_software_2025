export interface DropdownOption {
	value: any
	label: string
}

export interface ConfigOptionGroup {
	key: string
	label?: string
	description?: string
	layout: "row" | "column"
	options: ConfigOption[]
}

export interface ConfigOption {
	key: string
	label: string
	type: "string" | "number" | "boolean" | "dropdown" | "waypoints"
	defaultValue: any
	placeholder?: string
	description?: string
	category?: string
	// For number inputs
	min?: number
	max?: number
	step?: number
	// For dropdown inputs
	options?: DropdownOption[]
	// For compact display in groups
	compact?: boolean
}

export interface ConfigDevice {
	id: string
	name: string
	description?: string
	options: ConfigOption[]
	groups?: ConfigOptionGroup[]
}

export const configDevices: ConfigDevice[] = [
	{
		id: "autonav_vision_transformer",
		name: "Vision Transformer",
		options: [
			{
				key: "region_of_disinterest_offset",
				label: "Region of Disinterest Offset",
				type: "number",
				defaultValue: 0,
				min: 0,
				max: 100,
				step: 1
			},
			{
				key: "override_ramp",
				label: "Override Ramp",
				type: "boolean",
				defaultValue: false
			}
		],
		groups: [
			{
				key: "lower-thresholds-ground",
				label: "Lower Thresholds (Ground)",
				layout: "row",
				options: [
					{
						key: "lower_hue_ground",
						label: "Lower Hue Threshold",
						type: "number",
						defaultValue: 0,
						placeholder: "0-255",
						min: 0,
						max: 255,
						step: 1,
						category: "HSV"
					},
					{
						key: "lower_saturation_ground",
						label: "Lower Saturation Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
					},
					{
						key: "lower_value_ground",
						label: "Lower Value Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
					}
				]
			},
			{
				key: "upper-thresholds-ground",
				label: "Upper Thresholds (Ground)",
				layout: "row",
				options: [
					{
						key: "upper_hue_ground",
						label: "Upper Hue Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
						category: "HSV"
					},
					{
						key: "upper_saturation_ground",
						label: "Upper Saturation Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
						category: "HSV"
					},
					{
						key: "upper_value_ground",
						label: "Upper Value Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
						category: "HSV"
					}
				]
			},
			{
				key: "lower-thresholds-ramp",
				label: "Lower Thresholds (Ramp)",
				layout: "row",
				options: [
					{
						key: "lower_hue_ramp",
						label: "Lower Hue Threshold",
						type: "number",
						defaultValue: 0,
						placeholder: "0-255",
						min: 0,
						max: 255,
						step: 1,
						category: "HSV"
					},
					{
						key: "lower_saturation_ramp",
						label: "Lower Saturation Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
					},
					{
						key: "lower_value_ramp",
						label: "Lower Value Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
					}
				]
			},
			{
				key: "upper-thresholds-ramp",
				label: "Upper Thresholds (Ramp)",
				layout: "row",
				options: [
					{
						key: "upper_hue_ramp",
						label: "Upper Hue Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
						category: "HSV"
					},
					{
						key: "upper_saturation_ramp",
						label: "Upper Saturation Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
						category: "HSV"
					},
					{
						key: "upper_value_ramp",
						label: "Upper Value Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
					}
				]
			},
			{
				key: "blurs",
				label: "Blurring",
				layout: "row",
				options: [
					{
						key: "blur",
						label: "Blur Radius",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 100,
						step: 1,
						category: "Blurring"
					},
					{
						key: "blur_iterations",
						label: "Blur Iterations",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 100,
						step: 1,
						category: "Blurring"
					},
				]
			}
		]
	},
	{
		id: "autonav_nav_astar",
		name: "A* Navigation",
		options: [
			{
				key: "latitude_length",
				label: "Latitude Length",
				type: "number",
				defaultValue: 111086.2,
				min: 0,
				max: 1000000,
				step: 0.1,
			},
			{
				key: "longitude_length",
				label: "Longitude Length",
				type: "number",
				defaultValue: 91978.2,
				min: 0,
				max: 1000000,
				step: 0.1,
			},
			{
				key: "waypoint_pop_distance",
				label: "Waypoint Pop Distance",
				type: "number",
				defaultValue: 1.0,
				min: 0,
				max: 1000,
				step: 0.1,
			},
			{
				key: "waypoint_delay",
				label: "Waypoint Delay",
				type: "number",
				defaultValue: 10,
				min: 0,
				max: 1000,
				step: 0.1,
			},
			{
				key: "robot_y",
				label: "Robot Y Offset",
				type: "number",
				defaultValue: 70,
				min: -1000,
				max: 1000,
				step: 1,
			},
			{
				key: "use_only_waypoints",
				label: "Use Only Waypoints",
				type: "boolean",
				defaultValue: false,
			},
			{
				key: "waypoints",
				label: "Waypoints",
				type: "waypoints",
				defaultValue: [],
			}
		]
	},
	{
		id: "zemlin_path_resolver",
		name: "Path Resolver",
		options: [
			{
				key: "forward_speed",
				label: "Forward Speed",
				type: "number",
				defaultValue: 0.5,
				min: -10,
				max: 10,
				step: 0.1,
			},
			{
				key: "reverse_speed",
				label: "Reverse Speed",
				type: "number",
				defaultValue: -0.5,
				min: -10,
				max: 10,
				step: 0.1,
			},
			{
				key: "angular_aggressiveness",
				label: "Angular Aggressiveness",
				type: "number",
				defaultValue: 1.0,
				min: 0.1,
				max: 100.0,
				step: 0.1,	
			},
			{
				key: "max_angular_speed",
				label: "Max Angular Speed",
				type: "number",
				defaultValue: 1.0,
				min: 0.1,
				max: 10.0,
				step: 0.1,
			}
		],
		groups: [
			{
				key: "pure_pursuit",
				layout: "row",
				options: [
					{
						key: "radius_multiplier",
						label: "Radius Multiplier",
						type: "number",
						defaultValue: 1.0,
						min: 0.1,
						category: "Pure Pursuit",
					},
					{
						key: "radius_max",
						label: "Max Radius",
						type: "number",
						defaultValue: 1.0,
						min: 0.1,
						category: "Pure Pursuit",
					},
					{
						key: "radius_start",
						label: "Start Radius",
						type: "number",
						defaultValue: 0.5,
						min: 0.1,
						category: "Pure Pursuit",
					},
				]
			}
		]
	},
	{
		id: "zemlin_filters",
		name: "Position Filters",
		options: [
			{
				key: "latitude_length",
				label: "Latitude Length",
				type: "number",
				defaultValue: 111086.2,
				min: 0,
				max: 1000000,
				step: 0.1,
			},
			{
				key: "longitude_length",
				label: "Longitude Length",
				type: "number",
				defaultValue: 91978.2,
				min: 0,
				max: 1000000,
				step: 0.1,	
			},
			{
				key: "filter_type",
				label: "Filter Type",
				type: "dropdown",
				defaultValue: 0,
				options: [
					{ value: 0, label: "Dead Reckoning" },
					{ value: 1, label: "Particle Filter" },
					{ value: 2, label: "Bearing Filter" },
				]
			}
		]
	}
]
