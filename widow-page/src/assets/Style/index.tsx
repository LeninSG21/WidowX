import React from "react";
import { Typography, TypographyProps, makeStyles } from "@material-ui/core";

const styles = makeStyles({
	typography: ({ ubuntu }: { ubuntu?: boolean }) => {
		return {
			fontFamily: ubuntu ? "Ubuntu Mono" : "Lato",
		};
	},
});

interface TypProps extends TypographyProps {
	ubuntu?: boolean | null;
}

export const Typo: React.FC<TypProps> = ({
	className,
	ubuntu,
	children,
	...others
}) => {
	const classes = styles({ ubuntu });
	return (
		<Typography
			className={
				className
					? `${classes.typography} ${className}`
					: classes.typography
			}
			{...others}
		>
			{children}
		</Typography>
	);
};
