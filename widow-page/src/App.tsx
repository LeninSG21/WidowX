import React from "react";

import HomePage from "./components/Home";
import { BrowserRouter as Router, Route } from "react-router-dom";

function App() {
	return (
		<Router>
			<Route path="/" component={HomePage} />
		</Router>
	);
}

export default App;
