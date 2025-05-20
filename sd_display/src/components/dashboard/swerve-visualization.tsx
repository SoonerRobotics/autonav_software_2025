import { SwerveModule } from "../SwerveModule";

export function SwerveVisualization() {
    return (
        <div className="flex items-center justify-center rounded-md">
            <div className="grid grid-cols-2 w-full h-full">
                <SwerveModule id={"Front Left"} angle={0} />
                <SwerveModule id={"Front Right"} angle={0} />
                <SwerveModule id={"Back Left"} angle={0} />
                <SwerveModule id={"Back Right"} angle={0} />
            </div>
        </div>
    )
}
