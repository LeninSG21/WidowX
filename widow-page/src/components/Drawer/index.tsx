import React, { useContext } from "react";
import { ROUTES, MenuItem, language } from "../../constants/";
import { LangContext } from "../Home";
import {
	ListItem,
	ListItemText,
	List,
	Typography,
	Divider,
} from "@material-ui/core";
import { RouteComponentProps, withRouter, Link } from "react-router-dom";
import { useStyles } from "./style";

const menuItems = (lang: language): MenuItem[] => [
	{
		text: lang === "english" ? "Direct Kinematics" : "Cinemática Directa",
		route: ROUTES.DK,
	},
	{
		text: lang === "english" ? "Inverse Kinematics" : "Cinemática Inversa",
		route: ROUTES.IK,
	},
];

interface Props extends RouteComponentProps<any> {}

const DrawerMenu: React.FC<Props> = ({ history }) => {
	const lang = useContext(LangContext);
	const classes = useStyles();
	return (
		<div>
			<div className={`${classes.toolbar} ${classes.title}`}>
				{/* <div className={classes.title}> */}
				<Typography variant="h5">
					<Link to="/" className={classes.linkHome}>
						WidowX Library
					</Link>
				</Typography>
				{/* <Typography variant="caption">by</Typography>
				<br />
				<Typography variant="subtitle1">
					<a
						href="https://github.com/LeninSG21?tab=repositories"
						className={classes.linkHome}
						target="_blank"
					>
						LeninSG21
					</a>
				</Typography> */}
			</div>
			<Divider />
			<List>
				{menuItems(lang).map((item) => (
					<ListItem
						button
						key={item.text}
						onClick={() => history.push(item.route)}
					>
						<ListItemText primary={item.text} />
					</ListItem>
				))}
			</List>
		</div>
	);
};

export default withRouter(DrawerMenu);
