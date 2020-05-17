import React, { useState, createContext } from "react";
import { language, ROUTES } from "../../constants/";
import { RouteComponentProps, Route } from "react-router-dom";

import { useStyles } from "./style";
import {
	Hidden,
	Drawer,
	CssBaseline,
	AppBar,
	Toolbar,
	IconButton,
	MenuItem,
	Menu,
	Button,
} from "@material-ui/core";
import DrawerMenu from "../Drawer";
import MenuIcon from "@material-ui/icons/Menu";
import GitHubIcon from "@material-ui/icons/GitHub";
import TranslateIcon from "@material-ui/icons/Translate";
import ExpandMoreIcon from "@material-ui/icons/ExpandMore";
import ForwardKinematics from "../DK";
import { HomeView } from "./view";
import ArduinoLibrary from "../ArduinoLibrary";

export const LangContext = createContext<language>("english");

const HomePage: React.FC<RouteComponentProps<any>> = () => {
	const classes = useStyles();
	const [mobileOpen, setMobileOpen] = useState(false);
	const [lang, setLang] = useState<language>("english");

	return (
		<LangContext.Provider value={lang}>
			<div className={classes.root}>
				<CssBaseline />
				<AppBar position="fixed" className={classes.appBar}>
					<Toolbar>
						<IconButton
							color="inherit"
							aria-label="open drawer"
							edge="start"
							onClick={() => setMobileOpen(true)}
							className={classes.menuButton}
						>
							<MenuIcon />
						</IconButton>
						<div className={classes.grow} />

						<LanguageSelector lang={lang} setLang={setLang} />
						<IconButton
							onClick={() =>
								window.open(
									"https://github.com/LeninSG21/WidowX",
									"_blank"
								)
							}
							color="inherit"
						>
							<GitHubIcon />
						</IconButton>
					</Toolbar>
				</AppBar>
				<nav className={classes.drawer} aria-label="mailbox folders">
					<Hidden mdUp implementation="css">
						<Drawer
							variant="temporary"
							anchor="left"
							open={mobileOpen}
							onClose={() => setMobileOpen(false)}
							classes={{
								paper: classes.drawerPaper,
							}}
							ModalProps={{
								keepMounted: true, // Better open performance on mobile.
							}}
						>
							<DrawerMenu />
						</Drawer>
					</Hidden>
					<Hidden smDown implementation="css">
						<Drawer
							classes={{
								paper: classes.drawerPaper,
							}}
							variant="permanent"
							open
						>
							<DrawerMenu />
						</Drawer>
					</Hidden>
				</nav>
				<div className={classes.content}>
					{/* <Route exact path={"/"} render={() => <HomeView />} />
					<Route
						exact
						path={ROUTES.DK}
						render={() => <ForwardKinematics />}
					/> */}
					<Route
						exact
						path={ROUTES.CPP}
						render={() => <ArduinoLibrary />}
					/>
				</div>
			</div>
		</LangContext.Provider>
	);
};

export default HomePage;

interface Props {
	setLang: (lang: language) => void;
	lang: language;
}

const LanguageSelector: React.FC<Props> = ({ setLang, lang }) => {
	const [anchorEl, setAnchorEl] = React.useState<null | HTMLElement>(null);
	const classes = useStyles();

	const handleClick = (event: React.MouseEvent<HTMLButtonElement>) => {
		setAnchorEl(event.currentTarget);
	};

	const handleClose = () => {
		setAnchorEl(null);
	};

	return (
		<React.Fragment>
			{console.log(anchorEl)}
			<Button
				className={classes.select}
				startIcon={<TranslateIcon />}
				endIcon={<ExpandMoreIcon />}
				onClick={handleClick}
			>
				{lang}
			</Button>
			<Menu
				anchorEl={anchorEl}
				keepMounted
				open={Boolean(anchorEl)}
				onClose={handleClose}
			>
				<MenuItem
					onClick={() => {
						setLang("english");
						handleClose();
					}}
					style={{ width: "125px" }}
				>
					English
				</MenuItem>
				<MenuItem
					onClick={() => {
						setLang("spanish");
						handleClose();
					}}
					style={{ width: "125px" }}
				>
					Español
				</MenuItem>
			</Menu>
		</React.Fragment>
	);
};
