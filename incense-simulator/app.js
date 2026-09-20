(function () {
  "use strict";

  var canvas = document.getElementById("scene");
  var ctx = canvas.getContext("2d");
  var lightBtn = document.getElementById("lightBtn");
  var fanBtn = document.getElementById("fanBtn");
  var extinguishBtn = document.getElementById("extinguishBtn");
  var statusText = document.getElementById("statusText");
  var statusDot = document.getElementById("statusDot");
  var burnFill = document.getElementById("burnFill");
  var burnValue = document.getElementById("burnValue");
  var burnTime = document.getElementById("burnTime");
  var clock = document.getElementById("clock");
  var wishForm = document.getElementById("wishForm");
  var wishInput = document.getElementById("wishInput");
  var wishList = document.getElementById("wishList");
  var wishCount = document.getElementById("wishCount");
  var sessionCount = document.getElementById("sessionCount");
  var segmentButtons = Array.prototype.slice.call(document.querySelectorAll(".segment"));

  var dpr = 1;
  var width = 0;
  var height = 0;
  var lastTime = performance.now();
  var state = "idle";
  var stickCount = 1;
  var burn = 1;
  var burnDuration = 90000;
  var litAt = 0;
  var sessions = Number(localStorage.getItem("incenseSessions") || 0);
  var smoke = [];
  var sparks = [];
  var wind = 0;
  var pointerWind = 0;
  var previousPointerX = null;
  var wishes = [];

  sessionCount.textContent = String(sessions);

  function resize() {
    var rect = canvas.getBoundingClientRect();
    dpr = Math.min(window.devicePixelRatio || 1, 2);
    width = Math.max(1, rect.width);
    height = Math.max(1, rect.height);
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  }

  function updateClock() {
    var now = new Date();
    clock.textContent = now.toLocaleTimeString("zh-CN", {
      hour: "2-digit",
      minute: "2-digit",
      hour12: false
    });
  }

  function setStatus(next) {
    state = next;
    statusDot.classList.toggle("lit", state === "lit");
    if (state === "lit") {
      statusText.textContent = "香火清明";
      lightBtn.querySelector("span:last-child").textContent = "添香";
    } else if (state === "out") {
      statusText.textContent = "余烟未散";
      lightBtn.querySelector("span:last-child").textContent = "复燃";
    } else if (state === "done") {
      statusText.textContent = "一愿已成";
      lightBtn.querySelector("span:last-child").textContent = "添香";
    } else {
      statusText.textContent = "香未点燃";
      lightBtn.querySelector("span:last-child").textContent = "点香";
    }
  }

  function lightIncense() {
    if (state === "lit") {
      burn = 1;
    } else if (state === "done" || burn <= 0.02) {
      burn = 1;
    }
    litAt = performance.now();
    sessions += 1;
    sessionCount.textContent = String(sessions);
    localStorage.setItem("incenseSessions", String(sessions));
    setStatus("lit");
    addSparkBurst(12);
  }

  function extinguish() {
    if (state !== "lit") return;
    setStatus("out");
    wind += 0.8;
  }

  function fan() {
    wind += (Math.random() > 0.5 ? 1 : -1) * 2.2;
    if (state === "lit") addSparkBurst(5);
  }

  function addSparkBurst(amount) {
    var centers = getStickCenters();
    for (var i = 0; i < amount; i += 1) {
      var center = centers[i % centers.length];
      sparks.push({
        x: center,
        y: getEmberY() + Math.random() * 5,
        vx: (Math.random() - 0.5) * 34,
        vy: -20 - Math.random() * 45,
        life: 0.45 + Math.random() * 0.5,
        maxLife: 1
      });
    }
  }

  function getStickCenters() {
    var center = width * 0.46;
    if (stickCount === 1) return [center];
    return [center - 19, center, center + 19];
  }

  function getStickBottom() {
    return Math.min(height * 0.73, height - 118);
  }

  function getStickLength() {
    return Math.max(130, Math.min(255, height * 0.39));
  }

  function getEmberY() {
    return getStickBottom() - getStickLength() * burn;
  }

  function spawnSmoke() {
    if (smoke.length > 180) return;
    var centers = getStickCenters();
    for (var i = 0; i < centers.length; i += 1) {
      if (Math.random() < 0.72) {
        smoke.push({
          x: centers[i] + (Math.random() - 0.5) * 2,
          y: getEmberY() - 5,
          vx: (Math.random() - 0.5) * 5,
          vy: -12 - Math.random() * 9,
          age: 0,
          life: 3.8 + Math.random() * 2.8,
          size: 4 + Math.random() * 5,
          phase: Math.random() * Math.PI * 2
        });
      }
    }
  }

  function updateParticles(dt, now) {
    if (state === "lit") {
      burn -= dt / burnDuration;
      if (burn <= 0) {
        burn = 0;
        setStatus("done");
      } else {
        spawnSmoke();
        if (Math.random() < 0.035) addSparkBurst(1);
      }
    } else if (state === "out" && Math.random() < 0.1 && smoke.length < 80) {
      spawnSmoke();
    }

    wind *= Math.pow(0.12, dt / 3000);
    pointerWind *= Math.pow(0.08, dt / 1200);

    smoke = smoke.filter(function (particle) {
      particle.age += dt / 1000;
      particle.x += (particle.vx + wind * 18 + pointerWind * 10) * dt / 1000;
      particle.y += particle.vy * dt / 1000;
      particle.vx += Math.sin(particle.phase + particle.age * 2.2) * 0.12;
      particle.size += dt * 0.004;
      return particle.age < particle.life && particle.y > -30;
    });

    sparks = sparks.filter(function (spark) {
      spark.life -= dt / 1000;
      spark.x += spark.vx * dt / 1000;
      spark.y += spark.vy * dt / 1000;
      spark.vy += 25 * dt / 1000;
      return spark.life > 0;
    });

    var remaining = Math.max(0, Math.ceil((burn * burnDuration) / 1000));
    burnFill.style.transform = "scaleX(" + burn.toFixed(4) + ")";
    burnValue.textContent = Math.round(burn * 100) + "%";
    if (state === "lit") {
      burnTime.textContent = "清燃 " + Math.floor((now - litAt) / 1000) + " 秒";
    } else if (state === "done") {
      burnTime.textContent = "香尽愿留";
    } else if (state === "out") {
      burnTime.textContent = "余香 " + remaining + " 秒";
    } else {
      burnTime.textContent = "静候点香";
    }
  }

  function drawBackground() {
    ctx.fillStyle = "#101514";
    ctx.fillRect(0, 0, width, height);

    ctx.fillStyle = "#18201e";
    ctx.fillRect(0, height * 0.72, width, height * 0.28);

    ctx.fillStyle = "#252e2b";
    ctx.fillRect(0, height * 0.72, width, 2);

    ctx.fillStyle = "rgba(211, 170, 85, 0.08)";
    var center = width * 0.46;
    ctx.beginPath();
    ctx.moveTo(center - width * 0.22, height * 0.72);
    ctx.lineTo(center - 42, height * 0.18);
    ctx.lineTo(center + 42, height * 0.18);
    ctx.lineTo(center + width * 0.22, height * 0.72);
    ctx.closePath();
    ctx.fill();

    ctx.strokeStyle = "rgba(211, 170, 85, 0.12)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.arc(center, height * 0.34, 68, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(center, height * 0.34, 82, 0, Math.PI * 2);
    ctx.stroke();
  }

  function drawAltar() {
    var center = width * 0.46;
    var baseY = Math.min(height * 0.82, height - 72);
    var scale = Math.max(0.78, Math.min(1.2, width / 1100));

    ctx.fillStyle = "#0b0f0e";
    ctx.fillRect(center - 190 * scale, baseY + 25, 380 * scale, 18);
    ctx.fillStyle = "#442b28";
    ctx.fillRect(center - 165 * scale, baseY + 43, 330 * scale, 8);

    ctx.save();
    ctx.translate(center, baseY);
    ctx.scale(scale, scale);

    ctx.fillStyle = "#594a30";
    ctx.beginPath();
    ctx.ellipse(0, -4, 101, 23, 0, 0, Math.PI * 2);
    ctx.fill();

    ctx.fillStyle = "#9b783b";
    ctx.beginPath();
    ctx.moveTo(-88, -8);
    ctx.quadraticCurveTo(-78, 60, -42, 72);
    ctx.lineTo(42, 72);
    ctx.quadraticCurveTo(78, 60, 88, -8);
    ctx.closePath();
    ctx.fill();

    ctx.strokeStyle = "#d3aa55";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.ellipse(0, -8, 90, 22, 0, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = "#2e2a22";
    ctx.beginPath();
    ctx.ellipse(0, -8, 79, 15, 0, 0, Math.PI * 2);
    ctx.fill();

    ctx.strokeStyle = "#d3aa55";
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.arc(-92, 17, 19, Math.PI * 0.4, Math.PI * 1.6);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(92, 17, 19, -Math.PI * 0.6, Math.PI * 0.6);
    ctx.stroke();

    ctx.fillStyle = "#c39a4c";
    ctx.font = "24px STKaiti, KaiTi, serif";
    ctx.textAlign = "center";
    ctx.fillText("愿", 0, 39);
    ctx.restore();
  }

  function drawSticks(now) {
    var centers = getStickCenters();
    var bottom = getStickBottom();
    var fullLength = getStickLength();
    var top = bottom - fullLength * burn;

    centers.forEach(function (x, index) {
      ctx.lineCap = "round";
      ctx.lineWidth = 5;
      ctx.strokeStyle = "#c05742";
      ctx.beginPath();
      ctx.moveTo(x, bottom + 23);
      ctx.lineTo(x, top);
      ctx.stroke();

      ctx.lineWidth = 3;
      ctx.strokeStyle = "#a49c82";
      ctx.beginPath();
      ctx.moveTo(x, top);
      ctx.lineTo(x, Math.min(bottom, top + 10));
      ctx.stroke();

      if (state === "lit") {
        var glow = 4 + Math.sin(now * 0.008 + index) * 1.2;
        ctx.fillStyle = "rgba(255, 104, 70, 0.18)";
        ctx.beginPath();
        ctx.arc(x, top, glow * 3, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#ff765c";
        ctx.beginPath();
        ctx.arc(x, top, glow, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = "#ffd086";
        ctx.beginPath();
        ctx.arc(x, top - 1, 1.8, 0, Math.PI * 2);
        ctx.fill();
      }
    });
  }

  function drawSmoke() {
    ctx.save();
    ctx.lineCap = "round";
    smoke.forEach(function (particle) {
      var progress = particle.age / particle.life;
      var alpha = Math.sin(Math.min(1, progress) * Math.PI) * 0.2;
      ctx.strokeStyle = "rgba(212, 219, 214, " + alpha.toFixed(3) + ")";
      ctx.lineWidth = particle.size;
      ctx.beginPath();
      ctx.moveTo(particle.x, particle.y);
      ctx.quadraticCurveTo(
        particle.x - Math.sin(particle.phase + particle.age * 2) * 7,
        particle.y - 9,
        particle.x + Math.cos(particle.phase + particle.age) * 3,
        particle.y - 17
      );
      ctx.stroke();
    });
    ctx.restore();
  }

  function drawSparks() {
    sparks.forEach(function (spark) {
      var alpha = Math.max(0, spark.life / spark.maxLife);
      ctx.fillStyle = "rgba(255, 168, 83, " + alpha.toFixed(3) + ")";
      ctx.fillRect(spark.x, spark.y, 2, 2);
    });
  }

  function frame(now) {
    var dt = Math.min(40, now - lastTime);
    lastTime = now;
    updateParticles(dt, now);
    drawBackground();
    drawSmoke();
    drawAltar();
    drawSticks(now);
    drawSparks();
    requestAnimationFrame(frame);
  }

  lightBtn.addEventListener("click", lightIncense);
  extinguishBtn.addEventListener("click", extinguish);
  fanBtn.addEventListener("click", fan);

  segmentButtons.forEach(function (button) {
    button.addEventListener("click", function () {
      stickCount = Number(button.dataset.count);
      segmentButtons.forEach(function (item) {
        item.classList.toggle("active", item === button);
      });
      smoke = [];
    });
  });

  wishForm.addEventListener("submit", function (event) {
    event.preventDefault();
    var wish = wishInput.value.trim();
    if (!wish) {
      wishInput.focus();
      return;
    }
    wishes.unshift(wish);
    wishes = wishes.slice(0, 5);
    wishList.innerHTML = "";
    wishes.forEach(function (text) {
      var slip = document.createElement("div");
      slip.className = "wish-slip";
      slip.textContent = text;
      wishList.appendChild(slip);
    });
    wishCount.textContent = String(wishes.length);
    wishInput.value = "";
    if (state !== "lit") lightIncense();
  });

  canvas.addEventListener("pointermove", function (event) {
    if (previousPointerX !== null) {
      pointerWind += Math.max(-1.5, Math.min(1.5, (event.clientX - previousPointerX) * 0.035));
    }
    previousPointerX = event.clientX;
  });

  canvas.addEventListener("pointerleave", function () {
    previousPointerX = null;
  });

  canvas.addEventListener("click", function () {
    if (state !== "lit") lightIncense();
  });

  window.addEventListener("resize", resize);
  updateClock();
  setInterval(updateClock, 10000);
  resize();
  setStatus("idle");
  requestAnimationFrame(frame);
}());
