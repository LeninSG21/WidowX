import React, { useContext } from "react";
import ReactMarkdown from "react-markdown";
import { spanishFK, englishFK } from "./markdown";
import { LangContext } from "../Home";

const ForwardKinematics: React.FC = () => {
	const lang = useContext(LangContext);

	return (
		<ReactMarkdown source={lang === "english" ? englishFK : spanishFK} />
	);
};

export default ForwardKinematics;
