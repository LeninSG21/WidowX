import { makeStyles } from "@material-ui/core";
import { teal } from "@material-ui/core/colors";

const drawerWidth = 275;

export const useStyles = makeStyles((theme) => ({
	root: {
		display: "flex",
	},
	drawer: {
		[theme.breakpoints.up("sm")]: {
			width: drawerWidth,
			flexShrink: 0,
		},
	},
	drawerPaper: {
		width: drawerWidth,
		backgroundColor: teal[500],
		color: theme.palette.common.white,
	},
	appBar: {
		[theme.breakpoints.up("sm")]: {
			width: `calc(100% - ${drawerWidth}px)`,
			marginLeft: drawerWidth,
		},
		backgroundColor: teal[500],
		color: theme.palette.common.white,
	},
	menuButton: {
		marginRight: theme.spacing(2),
		[theme.breakpoints.up("sm")]: {
			display: "none",
		},
	},
	grow: {
		flexGrow: 1,
	},
	select: {
		color: theme.palette.common.white,
		marginRight: theme.spacing(1),
	},
	content: {
		marginLeft: theme.spacing(3),
		marginTop: theme.spacing(10),
	},
}));
