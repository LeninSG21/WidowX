import { makeStyles } from "@material-ui/core";

export const useStyles = makeStyles((theme) => ({
	intro: {
		fontSize: theme.typography.pxToRem(20),
		fontWeight: theme.typography.fontWeightRegular,
	},
	expansionPanel: {
		marginTop: theme.spacing(3),
		// textAlign: "center",
	},
	functionName: {
		fontSize: theme.typography.pxToRem(17),
		fontWeight: theme.typography.fontWeightBold,
	},
}));
