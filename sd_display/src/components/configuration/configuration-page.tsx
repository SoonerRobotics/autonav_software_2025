"use client"
import { Tabs, TabsContent, TabsList, TabsTrigger } from "@/components/ui/tabs"
import { Card, CardContent, CardHeader, CardTitle } from "@/components/ui/card"
import { DeviceConfigForm } from "@/components/configuration/device-config-form"
import { configDevices } from "@/lib/config-devices"

export function ConfigurationPage() {
  return (
    <div className="p-6 space-y-6">
      <div className="flex items-center justify-between">
        <h1 className="text-2xl font-bold">Device Configuration</h1>
      </div>

      <Card>
        <CardHeader>
          <CardTitle>Device Configuration</CardTitle>
        </CardHeader>
        <CardContent>
          <Tabs defaultValue="autonav_vision_transformer" className="w-full">
            <TabsList className="grid w-full grid-cols-4 lg:grid-cols-7">
              {configDevices.map((device) => (
                <TabsTrigger key={device.id} value={device.id} className="text-xs">
                  {device.name}
                </TabsTrigger>
              ))}
            </TabsList>

            {configDevices.map((device) => (
              <TabsContent key={device.id} value={device.id} className="mt-6">
                <DeviceConfigForm device={device} />
              </TabsContent>
            ))}
          </Tabs>
        </CardContent>
      </Card>
    </div>
  )
}
