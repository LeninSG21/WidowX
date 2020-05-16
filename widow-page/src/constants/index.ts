export type language = "english" | "spanish";

interface Routes {
	DK: "/direct_kinematics";
	IK: "/inverse_kinematics";
}

export const ROUTES: Routes = {
	DK: "/direct_kinematics",
	IK: "/inverse_kinematics",
};

export interface MenuItem {
	text: string;
	route: string;
}
