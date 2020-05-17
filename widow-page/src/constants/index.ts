export type language = "english" | "spanish";

interface Routes {
	DK: "/forward_kinematics";
	IK: "/inverse_kinematics";
	CPP: "/arduino_library";
	ROS: "/ros_package";
}

export const ROUTES: Routes = {
	DK: "/forward_kinematics",
	IK: "/inverse_kinematics",
	CPP: "/arduino_library",
	ROS: "/ros_package",
};

export interface MenuItem {
	text: string;
	route: string;
	icon?: JSX.Element;
}
