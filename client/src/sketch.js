// Communication between Arduino and p5.js: Based on Tom Igoe's lab examples
// Frozen brush: Adapted from Jason Labbe's https://openprocessing.org/sketch/413567

let portButton;
let pa1 = null;
let pa2 = null;
let pt1 = null;
let pt2 = null;

const padding = 10;
const maxLevel = 5;
const particles = [];

const serial = new p5.WebSerial();

function setup() {
  createCanvas(windowWidth, windowWidth);
  colorMode(HSB, 360);
  setupSerial();
  background(0);
}

function draw() {
  applyFadeEffect();
  displayParticles(particles);
}

function applyFadeEffect() {
  noStroke();
  fill(0, 30);
  rect(0, 0, width, height);
}

function displayParticles(particles) {
  // Move and spawn particles.
  // Remove any that is below the velocity threshold.
  for (let i = particles.length - 1; i > -1; i--) {
    const particle = particles[i];
    particle.move();
    if (particle.vel.mag() < 0.01) particles.splice(i, 1);
  }

  if (!particles.length) return;

  strokeWeight(0.1);

  const I = Delaunay.triangulate(particles.map((pt) => [pt.pos.x, pt.pos.y]));

  // Display triangles individually.
  for (let i = 0; i < I.length; i += 3) {
    // Collect particles that make this triangle.
    const p1 = particles[I[i]];
    const p2 = particles[I[i + 1]];
    const p3 = particles[I[i + 2]];

    const renderColor = color(165 + p1.life * 1.5, 360, 360);
    noStroke();
    fill(renderColor);

    triangle(p1.pos.x, p1.pos.y, p2.pos.x, p2.pos.y, p3.pos.x, p3.pos.y);
  }
}

class Particle {
  constructor(x, y, level = maxLevel) {
    this.level = level;
    this.life = 0;
    this.pos = new p5.Vector(x, y);
    this.vel = p5.Vector.random2D();
    this.vel.mult(map(this.level, 0, maxLevel, 5, 2));
    this.particles = particles;
  }
  move() {
    this.life++;
    this.vel.mult(0.9);
    this.pos.add(this.vel);
    if (this.life % 10 == 0 && this.level > 0) {
      this.level -= 1;
      const newParticle = new Particle(this.pos.x, this.pos.y, this.level - 1);
      particles.push(newParticle);
    }
  }
}

function transform(a1, a2, t1, t2) {
  // Apply forward kinematics to get the end effector position.
  const x1 = a1 * cos(t1);
  const y1 = a1 * sin(t1);
  const x2 = x1 + a2 * cos(t1 + t2);
  const y2 = y1 + a2 * sin(t1 + t2);

  // Scale the coordinates to fit the canvas.
  const domainYMax = a1 + a2;
  const domainYMin = 0;
  const domainXMax = a1 + a2;
  const domainXMin = -domainXMax;
  const scaleX = (t) => map(t, domainXMin, domainXMax, padding, width - padding);
  const scaleY = (t) => map(t, domainYMin, domainYMax, height - padding, padding);
  const scaledX1 = scaleX(x1);
  const scaledY1 = scaleY(y1);
  const scaledX2 = scaleX(x2);
  const scaledY2 = scaleY(y2);

  return [scaledX1, scaledY1, scaledX2, scaledY2];
}

function serialEvent() {
  const string = serial.readLine();
  if (!string) return;
  const [a1, a2, t1, t2] = string.split(",").map((d) => +d);
  const isMoved = pa1 !== a1 || pa2 !== a2 || pt1 !== t1 || pt2 !== t2;
  (pa1 = a1), (pa2 = a2), (pt1 = t1), (pt2 = t2);
  if (!isMoved) return;
  const [x1, y1, x2, y2] = transform(a1, a2, t1, t2);
  particles.push(new Particle(x1, y1));
  particles.push(new Particle(x2, y2));
}

function setupSerial() {
  if (!navigator.serial) alert("WebSerial is not supported in this browser. Try Chrome or MS Edge.");
  serial.getPorts();
  serial.on("noport", makePortButton);
  serial.on("portavailable", openPort);
  serial.on("requesterror", portError);
  serial.on("data", serialEvent);
  serial.on("close", makePortButton);
  navigator.serial.addEventListener("connect", portConnect);
  navigator.serial.addEventListener("disconnect", portDisconnect);
}

function makePortButton() {
  portButton = createButton("choose port");
  portButton.position(10, 10);
  portButton.mousePressed(choosePort);
}

function choosePort() {
  if (portButton) portButton.show();
  serial.requestPort();
}

function openPort() {
  const open = () => console.log("port open");
  serial.open().then(open);
  if (portButton) portButton.hide();
}

function portError(err) {
  alert("Serial port error: " + err);
}

function portConnect() {
  console.log("port connected");
  serial.getPorts();
}

function portDisconnect() {
  serial.close();
  console.log("port disconnected");
}

function closePort() {
  serial.close();
}
