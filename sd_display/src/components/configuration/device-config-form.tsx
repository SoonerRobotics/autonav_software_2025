"use client"

import { useEffect, useState } from "react"
import { Button } from "@/components/ui/button"
import { Input } from "@/components/ui/input"
import { Label } from "@/components/ui/label"
import { Switch } from "@/components/ui/switch"
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from "@/components/ui/select"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { Save, RotateCcw } from "lucide-react"
import type { ConfigDevice, ConfigOption, ConfigOptionGroup } from "@/lib/config-devices"
import { useSocket } from "@/providers/SocketProvider"

interface DeviceConfigFormProps {
  device: ConfigDevice
}

export function DeviceConfigForm({ device }: DeviceConfigFormProps) {
  const { configuration, api } = useSocket();

  const [values, setValues] = useState<Record<string, any>>(() => {
    const initialValues: Record<string, any> = {}
    device.options.forEach((option) => {
      initialValues[option.key] = option.defaultValue
    })
    return initialValues
  })

  useEffect(() => {
    for (const device_key in configuration) {
      if (device_key !== device.id) {
        continue;
      }
      
      const properties = configuration[device_key];
      for (const key in properties) {
        const value = properties[key];
        if (device.options.some((option) => option.key === key) || device.groups?.some((group) => group.options.some((option) => option.key === key))) {
          setValues((prev) => ({ ...prev, [key]: value }))
        }
      }
    }
  }, [configuration]);

  const [hasChanges, setHasChanges] = useState(false)

  const handleValueChange = (key: string, value: any) => {
    setValues((prev) => ({ ...prev, [key]: value }))
    setHasChanges(true)
  }

  const handleSave = () => {
    api.set_config(device.id, values);
    console.log(`Saving configuration for ${device.name}:`, values)
    setHasChanges(false)
  }

  const handleReset = () => {
    const resetValues: Record<string, any> = {}

    for (const device_key in configuration) {
      if (device_key !== device.id) {
        continue;
      }
      
      const properties = configuration[device_key];
      for (const key in properties) {
        const value = properties[key];
        if (device.options.some((option) => option.key === key) || device.groups?.some((group) => group.options.some((option) => option.key === key))) {
          resetValues[key] = value
        }
      }
    }

    setValues(resetValues)
    setHasChanges(false)
  }

  const renderConfigOption = (option: ConfigOption, isCompact = false) => {
    const value = values[option.key]
    const inputClassName = isCompact ? "h-8" : ""

    switch (option.type) {
      case "string":
        return (
          <div key={option.key} className={`space-y-1 ${isCompact ? "space-y-1" : "space-y-2"}`}>
            <Label htmlFor={option.key} className={isCompact ? "text-xs" : ""}>
              {option.label}
            </Label>
            <Input
              id={option.key}
              value={value || ""}
              onChange={(e) => handleValueChange(option.key, e.target.value)}
              placeholder={option.placeholder}
              className={inputClassName}
            />
            {option.description && !isCompact && <p className="text-xs text-muted-foreground">{option.description}</p>}
          </div>
        )

      case "number":
        return (
          <div key={option.key} className={`space-y-1 ${isCompact ? "space-y-1" : "space-y-2"}`}>
            <Label htmlFor={option.key} className={isCompact ? "text-xs" : ""}>
              {option.label}
            </Label>
            <Input
              id={option.key}
              type="number"
              value={value || ""}
              onChange={(e) => handleValueChange(option.key, Number.parseFloat(e.target.value) || 0)}
              placeholder={option.placeholder}
              min={option.min}
              max={option.max}
              step={option.step}
              className={inputClassName}
            />
            {option.description && !isCompact && <p className="text-xs text-muted-foreground">{option.description}</p>}
          </div>
        )

      case "boolean":
        return (
          <div
            key={option.key}
            className={`flex items-center justify-between ${isCompact ? "space-y-1" : "space-y-2"}`}
          >
            <div className="space-y-1">
              <Label htmlFor={option.key} className={isCompact ? "text-xs" : ""}>
                {option.label}
              </Label>
              {option.description && !isCompact && (
                <p className="text-xs text-muted-foreground">{option.description}</p>
              )}
            </div>
            <Switch
              id={option.key}
              checked={value || false}
              onCheckedChange={(checked) => handleValueChange(option.key, checked)}
            />
          </div>
        )

      case "dropdown":
        return (
          <div key={option.key} className={`space-y-1 ${isCompact ? "space-y-1" : "space-y-2"}`}>
            <Label htmlFor={option.key} className={isCompact ? "text-xs" : ""}>
              {option.label}
            </Label>
            <Select value={value || ""} onValueChange={(newValue) => handleValueChange(option.key, newValue)}>
              <SelectTrigger className={inputClassName}>
                <SelectValue placeholder={option.placeholder} />
              </SelectTrigger>
              <SelectContent>
                {option.options?.map((opt) => (
                  <SelectItem key={opt.value} value={opt.value}>
                    {opt.label}
                  </SelectItem>
                ))}
              </SelectContent>
            </Select>
            {option.description && !isCompact && <p className="text-xs text-muted-foreground">{option.description}</p>}
          </div>
        )

      default:
        return null
    }
  }

  const renderOptionGroup = (group: ConfigOptionGroup) => {
    return (
      <div key={group.key} className="space-y-3 p-4 border rounded-lg bg-muted/30">
        <div>
          <h4 className="text-sm font-medium">{group.label}</h4>
          {group.description && <p className="text-xs text-muted-foreground mt-1">{group.description}</p>}
        </div>
        <div
          className={
            group.layout === "row"
              ? "grid grid-cols-2 md:grid-cols-3 lg:grid-cols-6 gap-3"
              : "grid grid-cols-1 md:grid-cols-2 gap-3"
          }
        >
          {group.options.map((option) => renderConfigOption(option, option.compact))}
        </div>
      </div>
    )
  }

  // Group options by category if they exist, excluding grouped options
  const groupedOptions = device.options.reduce(
    (acc, option) => {
      // Skip options that are part of a group
      const isInGroup = device.groups?.some((group) =>
        group.options.some((groupOption) => groupOption.key === option.key),
      )

      if (!isInGroup) {
        const category = option.category || "General"
        if (!acc[category]) {
          acc[category] = []
        }
        acc[category].push(option)
      }
      return acc
    },
    {} as Record<string, ConfigOption[]>,
  )

  // Group the option groups by category
  const groupsByCategory =
    device.groups?.reduce(
      (acc, group) => {
        // Determine category from the first option in the group, or use "General"
        const category = group.options[0]?.category || "General"
        if (!acc[category]) {
          acc[category] = []
        }
        acc[category].push(group)
        return acc
      },
      {} as Record<string, ConfigOptionGroup[]>,
    ) || {}

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <div>
          <h2 className="text-lg font-semibold">{device.name}</h2>
          <p className="text-sm text-muted-foreground">{device.description}</p>
        </div>
        <div className="flex gap-2">
          <Button variant="outline" onClick={handleReset} disabled={!hasChanges}>
            <RotateCcw className="h-4 w-4 mr-2" />
            Reset
          </Button>
          <Button onClick={handleSave} disabled={!hasChanges}>
            <Save className="h-4 w-4 mr-2" />
            Save Changes
          </Button>
        </div>
      </div>

      <div className="grid gap-6">
        {Object.entries(groupedOptions).map(([category, options]) => (
          <Card key={category}>
            <CardHeader>
              <CardTitle className="text-base">{category}</CardTitle>
            </CardHeader>
            <CardContent className="space-y-4">
              <div className="grid gap-4 md:grid-cols-2">{options.map((option) => renderConfigOption(option))}</div>
              {groupsByCategory[category] && (
                <div className="space-y-4">{groupsByCategory[category].map(renderOptionGroup)}</div>
              )}

              {Object.entries(groupsByCategory).map(([category, groups]) => (
                <div key={category} className="space-y-4">
                  <h3 className="text-lg font-semibold">{category}</h3>
                  {groups.map(renderOptionGroup)}
                </div>
              ))}
            </CardContent>
          </Card>
        ))}

        {/* Render groups that don't have a category */}
        {device.groups?.filter((group) => !group.options[0]?.category).map(renderOptionGroup)}
      </div>
    </div>
  )
}
