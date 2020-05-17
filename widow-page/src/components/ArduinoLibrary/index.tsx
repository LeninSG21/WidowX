import React, { useContext, useEffect, useState } from "react";
import ReactMarkdown from "react-markdown";
import { spanishAL, englishAL } from "./markdown";
import { LangContext } from "../Home";
import { Typo } from "../../assets/Style";
import { useStyles } from "./style";
import {
	ExpansionPanel,
	ExpansionPanelSummary,
	ExpansionPanelDetails,
	Grid,
} from "@material-ui/core";
import ExpandMoreIcon from "@material-ui/icons/ExpandMore";

const ArduinoLibrary: React.FC = () => {
	const lang = useContext(LangContext);
	const [text, setText] = useState(englishAL);
	const classes = useStyles();
	useEffect(() => {
		if (lang === "english") {
			setText(englishAL);
		} else {
			setText(spanishAL);
		}
	}, [lang]);
	return (
		<div>
			<Typo variant="h3" ubuntu>
				<u>{text.title}</u>
			</Typo>
			<Typo className={classes.intro} align="justify">
				<ReactMarkdown source={text.intro} />
			</Typo>
			<Typo variant="h4" id="functions">
				<u>
					<i>{text.s1}</i>
				</u>
			</Typo>
			<Grid container className={classes.expansionPanel} spacing={2}>
				{text.functions.map((func) => (
					<Grid item xs={12} sm={6} lg={4}>
						<ExpansionPanel>
							<ExpansionPanelSummary
								id={func.name}
								expandIcon={<ExpandMoreIcon />}
							>
								<Typo className={classes.functionName} ubuntu>
									{func.name}
								</Typo>
							</ExpansionPanelSummary>
							<ExpansionPanelDetails>
								<Typo align="justify">{func.description}</Typo>
							</ExpansionPanelDetails>
						</ExpansionPanel>
					</Grid>
				))}
			</Grid>
		</div>
	);
};

export default ArduinoLibrary;
