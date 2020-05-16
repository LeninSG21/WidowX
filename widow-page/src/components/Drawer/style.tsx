import { makeStyles, ThemeProvider } from "@material-ui/core";

export const useStyles = makeStyles((theme) => ({
	title: {
		textAlign: "center",
		paddingTop: theme.spacing(2),
	},
	linkHome: {
		textDecoration: "none",
		color: theme.palette.common.black,
	},
	toolbar: theme.mixins.toolbar,
}));
