let rosbridgeModule = await import(`${base_url}/js/modules/rosbridge.js`);

let rosbridge = rosbridgeModule.rosbridge;
let packages = [];

async function runRosWTF() {
	const wtfService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/roswtf",
		serviceType: "std_srvs/srv/Trigger",
	});

	return new Promise((resolve, reject) => {
		wtfService.callService(new ROSLIB.ServiceRequest(), (result) => {
			// The ASCII code for escape is 27, represented in hexadecimal as \x1B
			resolve(result.message.replace(/\x1B\[1m/g, '').replace(/\x1B\[0m/g, '').replace(/ /g, '\u00a0'));
		}, (error) => {
			reject(error);
			alert(error);
		});
	});
}

async function getExecutables(pkg_name) {
	const getExecutablesService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/list_executables",
		serviceType: "vizanti_msgs/srv/ListExecutables",
	});

	return new Promise((resolve, reject) => {
		getExecutablesService.callService(new ROSLIB.ServiceRequest({package : pkg_name}), (result) => {
			resolve(result.executables);
		}, (error) => {
			reject(error);
		});
	});
}

async function getPackages() {
	const getPackagesService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/list_packages",
		serviceType: "vizanti_msgs/srv/ListPackages",
	});

	return new Promise((resolve, reject) => {
		getPackagesService.callService(new ROSLIB.ServiceRequest(), (result) => {
			resolve(result.packages);
		}, (error) => {
			reject(error);
		});
	});
}

async function startNode(command) {
	const startService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/node/start",
		serviceType: "vizanti_msgs/srv/ManageNode",
	});

	return new Promise((resolve, reject) => {
		startService.callService(new ROSLIB.ServiceRequest({ node: command }), (result) => {
			resolve(result);
		}, (error) => {
			reject(error);
		});
	});
}

async function killNode(name) {
	const killService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/node/kill",
		serviceType: "vizanti_msgs/srv/ManageNode",
	});

	return new Promise((resolve, reject) => {
		killService.callService(new ROSLIB.ServiceRequest({ node: name }), (result) => {
			resolve(result);
		}, (error) => {
			reject(error);
			alert(error);
		});
	});
}

async function killLaunch(name) {
	const killService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/launch/kill",
		serviceType: "vizanti_msgs/srv/ManageLaunch",
	});

	return new Promise((resolve, reject) => {
		killService.callService(new ROSLIB.ServiceRequest({ launch: name }), (result) => {
			resolve(result);
		}, (error) => {
			reject(error);
			alert(error);
		});
	});
}

async function nodeInfo(name) {
	const infoService = new ROSLIB.Service({
		ros: rosbridge.ros,
		name: "/vizanti/node/info",
		serviceType: "vizanti_msgs/srv/ManageNode",
	});

	return new Promise((resolve, reject) => {
		infoService.callService(new ROSLIB.ServiceRequest({ node: name }), (result) => {
			resolve(result.message.replace(/ /g, '\u00a0'));
		}, (error) => {
			reject(error);
			alert(error);
		});
	});
}

const icon = document.getElementById("{uniqueID}_icon").getElementsByTagName('img')[0];
const nodeDiv = document.getElementById('{uniqueID}_nodelist');
const launchDiv = document.getElementById('{uniqueID}_launchlist');

const node_contextTitle = document.getElementById('{uniqueID}_node_context_title');
const launch_contextTitle = document.getElementById('{uniqueID}_launch_context_title');
const killnodeButton = document.getElementById('{uniqueID}_nodekill');
const infoText = document.getElementById('{uniqueID}_rosnode_info');

let currentNode = "";
let currentLaunch = "";
killnodeButton.addEventListener('click', async () => {
	if(await confirm("Do you really want to kill "+currentNode+"?")){
		console.log("Killing "+currentNode);
		await killNode(currentNode);
		closeModal("{uniqueID}_node_contextmodal");
		updateNodeList();
	}
});

async function updateNodeList(){
	let result = await rosbridge.get_all_nodes();

	nodeDiv.innerHTML = '';
	result.nodes.forEach(node => {

		if(node.includes("vizanti"))
			return;

		const nodeBox = document.createElement('div');
		nodeBox.className = 'node-box';
	
		const nodeName = document.createElement('span');
		nodeName.textContent = node;
	
		nodeBox.addEventListener('click', async () => {
			infoText.innerText = "Waiting for rosnode info...";

			currentNode = node;
			node_contextTitle.innerText = node;
			openModal("{uniqueID}_node_contextmodal");

			infoText.innerText = await nodeInfo(node);
		});
	
		nodeBox.appendChild(nodeName);
		nodeDiv.appendChild(nodeBox);
	});
}

async function updateLaunchList(){
	let result = await rosbridge.get_all_launches();

	launchDiv.innerHTML = '';
	result.launches.forEach(launch => {

		if(launch.includes("vizanti_serve"))
			return;

		const launchBox = document.createElement('div');
		launchBox.className = 'launch-box';
	
		const launchName = document.createElement('span');
		launchName.textContent = launch;
	
		launchBox.addEventListener('click', async () => {

			currentLaunch = launch;
			launch_contextTitle.innerText = launch;
			openModal("{uniqueID}_launch_contextmodal");
		});
	
		launchBox.appendChild(launchName);
		launchDiv.appendChild(launchBox);
	});
}
// roswtf

const wtfText = document.getElementById('{uniqueID}_roswtf_data');
const wtfButton = document.getElementById('{uniqueID}_roswtf');

wtfButton.addEventListener('click', async () => {
	wtfText.innerText = "Waiting for roswtf report (might take several seconds)...";
	openModal("{uniqueID}_roswtfmodal");
	wtfText.innerText = await runRosWTF();
});


// package picking
const executeButton = document.getElementById('{uniqueID}_execute');

const typeBox = document.getElementById('{uniqueID}_type');
const packageBox = document.getElementById('{uniqueID}_package');
const packageDataList = document.getElementById('{uniqueID}_package_datalist');
const nameBox = document.getElementById('{uniqueID}_name');
const paramBox = document.getElementById('{uniqueID}_param');

packageBox.addEventListener('change', async function(e) {
	const val = packageBox.value;

	if(packages.includes(val)) {
		let executables = await getExecutables(val);
		let nodelist = "";
		for (const exe of executables) {
			nodelist += "<option value='"+exe+"'>"+exe+"</option>"
		}
		nameBox.innerHTML = nodelist;

		if(nameBox.value.includes("launch")){
			typeBox.value = "ros2 launch";
		}else{
			typeBox.value = "ros2 run";
		}
	}
});

nameBox.addEventListener('change', async function(e) {
	if(nameBox.value.includes("launch")){
		typeBox.value = "ros2 launch";
	}else{
		typeBox.value = "ros2 run";
	}
});

async function updatePackageList(){
	packages = await getPackages();
	packages.forEach(pkg => {
		let option = document.createElement('option');
		option.value = pkg;
		packageDataList.appendChild(option);
	});
}

executeButton.addEventListener('click', async () => {
	if(await confirm("Do you really want to run '"+typeBox.value+" "+packageBox.value+" "+nameBox.value+" "+paramBox.value+"'?")){
		console.log("Executing "+typeBox.value+" "+packageBox.value+" "+nameBox.value+" "+paramBox.value);
		let response = await startNode(typeBox.value+" "+packageBox.value+" "+nameBox.value+" "+paramBox.value);
		alert(response.message);
		setTimeout(updateNodeList,1000);
	}
});

icon.addEventListener("click", updateNodeList);

updateNodeList();
updateLaunchList();
updatePackageList();

console.log("Ros Process Manager Loaded {uniqueID}")
