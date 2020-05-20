import React from "react";
import "./App.css";
import { BrowserRouter as Router, Route } from "react-router-dom";
import HomePage from "./components/Home";

function App() {
	return (
		<Router>
			<Route path="/WidowX" render={(props) => <HomePage {...props} />} />
		</Router>
	);
}

export default App;
