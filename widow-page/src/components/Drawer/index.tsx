import React, { useContext } from "react";
import { ROUTES, MenuItem, language } from "../../constants/";
import { LangContext } from "../Home";
import {
	ListItem,
	ListItemText,
	List,
	Typography,
	Divider,
	ListItemIcon,
} from "@material-ui/core";
import { RouteComponentProps, withRouter, Link } from "react-router-dom";
import { useStyles } from "./style";
import ForwardIcon from "@material-ui/icons/Forward";
import SwapHorizIcon from "@material-ui/icons/SwapHoriz";
import AppsIcon from "@material-ui/icons/Apps";
import AllInclusiveIcon from "@material-ui/icons/AllInclusive";

const menuItems = (lang: language): MenuItem[] => [
	{
		text: lang === "english" ? "Forward Kinematics" : "Cinemática Directa",
		route: ROUTES.DK,
		icon: <ForwardIcon />,
	},
	{
		text: lang === "english" ? "Inverse Kinematics" : "Cinemática Inversa",
		route: ROUTES.IK,
		icon: <SwapHorizIcon />,
	},
	{
		text: lang === "english" ? "Arduino Library" : "Librería de Arduino",
		route: ROUTES.CPP,
		icon: <AllInclusiveIcon />,
	},
	{
		text: lang === "english" ? "ROS Package" : "Paquete de ROS",
		route: ROUTES.ROS,
		icon: <AppsIcon />,
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
						{item.icon && (
							<ListItemIcon className={classes.menuIcon}>
								{item.icon}
							</ListItemIcon>
						)}
						<ListItemText
							primary={
								<Typography variant="button">
									{item.text}
								</Typography>
							}
						/>
					</ListItem>
				))}
			</List>
		</div>
	);
};

export default withRouter(DrawerMenu);
