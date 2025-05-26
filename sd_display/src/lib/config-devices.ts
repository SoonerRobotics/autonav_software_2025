export interface DropdownOption {
	value: string
	label: string
}

export interface ConfigOptionGroup {
	key: string
	label: string
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
		],
		groups: [
			{
				key: "lower-thresholds",
				label: "Lower Thresholds",
				layout: "row",
				options: [
					{
						key: "lower_hue",
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
						key: "lower_saturation",
						label: "Lower Saturation Threshold",
						placeholder: "0-255",
						type: "number",
						defaultValue: 0,
						min: 0,
						max: 255,
						step: 1,
					},
					{
						key: "lower_value",
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
				key: "upper-thresholds",
				label: "Upper Thresholds",
				layout: "row",
				options: [
					{
						key: "upper_hue",
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
						key: "upper_saturation",
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
						key: "upper_value",
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
	}
]
