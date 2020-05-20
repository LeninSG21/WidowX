export type language = "english" | "spanish";

interface Routes {
	DK: "/WidowX/forward_kinematics";
	IK: "/WidowX/inverse_kinematics";
	CPP: "/WidowX/arduino_library";
	ROS: "/WidowX/ros_package";
}

export const ROUTES: Routes = {
	DK: "/WidowX/forward_kinematics",
	IK: "/WidowX/inverse_kinematics",
	CPP: "/WidowX/arduino_library",
	ROS: "/WidowX/ros_package",
};

export interface MenuItem {
	text: string;
	route: string;
	icon?: JSX.Element;
}
