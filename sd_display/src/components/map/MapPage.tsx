import { useEffect } from "react"

import { Feature } from "ol";
import Map from "ol/Map";
import View from "ol/View";
import { Point } from "ol/geom";
import TileLayer from "ol/layer/Tile";
import VectorLayer from "ol/layer/Vector";
import { fromLonLat } from "ol/proj";
import VectorSource from "ol/source/Vector";
import Icon from "ol/style/Icon";
import Style from "ol/style/Style";
import XYZ from "ol/source/XYZ";
import { useSocket } from "@/providers/SocketProvider";

export default function MapPage() {
    const { configuration } = useSocket();

    useEffect(() => {
        const layers: any[] = [
                new TileLayer({
                    source: new XYZ({
                        url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
                        maxZoom: 19,
                    }),
                }),
        ];

        const waypoints = configuration?.autonav_nav_astar?.waypoints?.[0] || [];
        console.log("Waypoints:", waypoints);
        if (waypoints && waypoints?.length > 0) {
            const features = waypoints?.map((waypoint: any) => {
                return new Feature({
                    geometry: new Point(fromLonLat([waypoint[1], waypoint[0]])),
                    name: waypoint.name,
                });
            });

            const vectorSource = new VectorSource({
                features: features,
            });

            const vectorLayer = new VectorLayer({
                source: vectorSource,
                style: new Style({
                    image: new Icon({
                        src: "https://openlayers.org/en/v6.5.0/examples/data/icon.png",
                        anchor: [0.5, 1],
                    }),
                })
            });

            layers.push(vectorLayer);
        }

        const map = new Map({
            target: "map",
            layers: layers,
            view: new View({
                center: fromLonLat([-97.44199275186871, 35.21014185571627]),
                zoom: 17,
            }),
        });

        // fit map if waypoints are present
        if (waypoints && waypoints.length > 0) {
            const extent = layers[1].getSource().getExtent();
            map.getView().fit(extent, { duration: 1000 });
        }

        return () => {
            map.setTarget(undefined); // Clean up the map instance
        }
    }, []);

    return (
        <div className="h-full w-full flex flex-col items-center justify-center">
            <div id="map" className="w-full h-full"></div>
        </div>
    )
}