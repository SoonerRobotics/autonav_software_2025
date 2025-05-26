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
	type: "string" | "number" | "boolean" | "dropdown"
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
]
