let width = 100;
let height = 100;

const numAgents = 100;
const showPath = true
const adjMtx = [[0, 1], [1, 0]]
const speedstep = 10

var agents = [];

function initAgents() {
  const margin = 0.6
  for (var i = 0; i < numAgents; i += 1) {
    agents[agents.length] = {
      x: Math.random() * width * 0.6 + width * 0.2,
      // y: Math.random() * height * 0.6 + height * 0.2, 
      y: height/2,
      dx: 0,
      dy: 0,
      path: [],
    };
  }
}

function distance(agent1, agent2) {
  return Math.sqrt(
    (agent1.x - agent2.x) * (agent1.x - agent2.x) +
      (agent1.y - agent2.y) * (agent1.y - agent2.y),
  );
}

function sizeCanvas() {
  const canvas = document.getElementById("converge_canvas");
  width = window.innerWidth;
  height = window.innerHeight;
  canvas.width = width;
  canvas.height = height;
}

function keepWithinBounds(agent) {
  const margin = 0.95;

  if (agent.x > width * margin) {
    agent.dx = 0;
  }
  if (agent.x < width - width*margin) {
    agent.dx = -0
  }
  if (agent.y < height*(1-margin)) {
    agent.dy = 0;
  }
  if (agent.y > height*margin) {
    agent.dy = -0;
  }
}


function drawAgent(ctx, agent) {
  const angle = Math.atan2(agent.dy, agent.dx);
  ctx.translate(agent.x, agent.y);
  ctx.rotate(angle);
  ctx.translate(-agent.x, -agent.y);
  ctx.fillStyle = "#a2b9bc";
  ctx.beginPath();
  ctx.moveTo(agent.x, agent.y);
  ctx.lineTo(agent.x - 15, agent.y + 5);
  ctx.lineTo(agent.x - 15, agent.y - 5);
  ctx.lineTo(agent.x, agent.y);
  ctx.fill();
  ctx.setTransform(1, 0, 0, 1, 0, 0); // I

  if (showPath) {
    ctx.strokeStyle = "#878f99";
    ctx.beginPath();
    ctx.moveTo(agent.path[0][0], agent.path[0][1]);
    for (const point of agent.path) {
      ctx.lineTo(point[0], point[1]);
    }
    ctx.stroke();
  }
}

function agentDynamics(agent) {
  keepWithinBounds(agent);
  agent.x += agent.dx;
  agent.y += agent.dy;
  agent.path.push([agent.x, agent.y])
  agent.path = agent.path.slice(-500);
}

function animationLoop() {
  for (let agent of agents) {
    for (let agent2 of agents) {
      agent.dx += (agent2.x^2 - agent.x^2)^(1/(agent2.x + agent.x));
    }
    agentDynamics(agent);
    // agent.x += agent.dx;
    // agent.y += agent.dy;
  }

  const ctx = document.getElementById("converge_canvas").getContext("2d");
  ctx.clearRect(0, 0, width, height);
  for (let agent of agents) {
    drawAgent(ctx, agent);
  }

  window.requestAnimationFrame(animationLoop);
}

window.onload = () => {
  window.addEventListener("resize", sizeCanvas, false);
  sizeCanvas();

  initAgents();

  window.requestAnimationFrame(animationLoop);
};