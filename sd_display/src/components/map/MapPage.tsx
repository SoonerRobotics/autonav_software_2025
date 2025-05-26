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

export default function MapPage() {
    useEffect(() => {
        const map = new Map({
            target: "map",
            layers: [
                new TileLayer({
                    source: new XYZ({
                        url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
                        maxZoom: 19,
                    }),
                }),
            ],
            view: new View({
                center: fromLonLat([-97.44199275186871, 35.21014185571627]),
                zoom: 17,
            }),
        });

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