import { useEffect, useRef, useState } from "react";
import { Button } from "../ui/button";
import Map from "ol/Map";
import View from "ol/View";
import TileLayer from "ol/layer/Tile";
import VectorLayer from "ol/layer/Vector";
import VectorSource from "ol/source/Vector";
import XYZ from "ol/source/XYZ";
import { fromLonLat, toLonLat } from "ol/proj";
import { Feature } from "ol";
import Point from "ol/geom/Point";
import Style from "ol/style/Style";
import Icon from "ol/style/Icon";
import { Draw, Modify, Select } from "ol/interaction";
import { click } from "ol/events/condition";

export interface WaypointEditorProps {
    initialWaypoints?: Array<{ latitude: number; longitude: number }>;
    onSave?: (waypoints: Array<{ latitude: number; longitude: number }>) => void;
}

export default function WaypointEditor(props: WaypointEditorProps) {
    const [editing, setEditing] = useState(false);
    const vectorSourceRef = useRef(new VectorSource());

    useEffect(() => {
        const features = (props.initialWaypoints ?? []).map((wp) => {
            const feature = new Feature({
                geometry: new Point(fromLonLat([wp.longitude, wp.latitude])),
            });
            feature.setStyle(new Style({
                image: new Icon({
                    src: "https://openlayers.org/en/v6.5.0/examples/data/icon.png",
                    anchor: [0.5, 1],
                }),
            }));
            return feature;
        });

        vectorSourceRef.current.addFeatures(features);

        const vectorLayer = new VectorLayer({
            source: vectorSourceRef.current,
        });

        const map = new Map({
            target: "map",
            layers: [
                new TileLayer({
                    source: new XYZ({
                        url: "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}",
                    }),
                }),
                vectorLayer,
            ],
            view: new View({
                center: fromLonLat([-97.44199275186871, 35.21014185571627]),
                zoom: 17,
            }),
        });

        const modify = new Modify({ source: vectorSourceRef.current });
        map.addInteraction(modify);

        const draw = new Draw({ source: vectorSourceRef.current, type: "Point" });
        map.addInteraction(draw);

        const select = new Select({ condition: click });
        map.addInteraction(select);
        select.on("select", (e) => {
            e.selected.forEach((feature) => vectorSourceRef.current.removeFeature(feature));
        });

        // Fit view to features if there are any
        if (features.length > 0) {
            const extent = vectorSourceRef.current.getExtent();
            map.getView().fit(extent, { padding: [50, 50, 50, 50], maxZoom: 18 });
        }

        return () => {
            map.setTarget(undefined);
        };
    }, [editing, props.initialWaypoints]);

    const saveWaypoints = () => {
        const features = vectorSourceRef.current.getFeatures();
        const waypoints = features.map((f) => {
            const coords = (f.getGeometry() as Point).getCoordinates();
            const [lon, lat] = toLonLat(coords);
            return { latitude: lat, longitude: lon };
        });
        props.onSave?.(waypoints);
        setEditing(false);
    };

    return (
        <div className="w-full h-full flex flex-col">
            <Button variant="outline" onClick={() => setEditing(!editing)} className="mb-2 w-40">
                {editing ? "Close Editor" : "Edit Waypoints"}
            </Button>
            <div className={`flex-col gap-2 ${editing ? "flex" : "hidden"}`}>
                <div id="map" className="w-full h-[500px] border" />
                <Button onClick={saveWaypoints}>Save Waypoints</Button>
            </div>
        </div>
    );
}
