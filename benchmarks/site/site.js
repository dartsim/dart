const state = {
  data: null,
  grouped: {},
  chart: null,
};

const benchmarkSelect = document.getElementById("benchmarkSelect");
const runnerSelect = document.getElementById("runnerSelect");
const caseSelect = document.getElementById("caseSelect");
const updatedAt = document.getElementById("updatedAt");
const reportCount = document.getElementById("reportCount");
const latestValue = document.getElementById("latestValue");
const latestMeta = document.getElementById("latestMeta");
const deltaValue = document.getElementById("deltaValue");
const deltaMeta = document.getElementById("deltaMeta");
const emptyState = document.getElementById("emptyState");

const unitScale = [
  { unit: "ns", scale: 1e-9 },
  { unit: "us", scale: 1e-6 },
  { unit: "ms", scale: 1e-3 },
  { unit: "s", scale: 1 },
];

function formatSeconds(seconds) {
  if (seconds === null || Number.isNaN(seconds)) {
    return "--";
  }
  for (const { unit, scale } of unitScale) {
    const value = seconds / scale;
    if (value < 1000 || unit === "s") {
      return `${value.toFixed(3)} ${unit}`;
    }
  }
  return `${seconds.toFixed(3)} s`;
}

function formatPercent(delta) {
  if (delta === null || Number.isNaN(delta)) {
    return "--";
  }
  const sign = delta > 0 ? "+" : "";
  return `${sign}${delta.toFixed(2)}%`;
}

function groupReports(reports) {
  const grouped = {};
  for (const report of reports) {
    if (!report.slug || !report.runner_id) {
      continue;
    }
    if (!grouped[report.slug]) {
      grouped[report.slug] = {};
    }
    if (!grouped[report.slug][report.runner_id]) {
      grouped[report.slug][report.runner_id] = [];
    }
    grouped[report.slug][report.runner_id].push(report);
  }

  for (const slug of Object.keys(grouped)) {
    for (const runner of Object.keys(grouped[slug])) {
      grouped[slug][runner].sort((a, b) => {
        return String(a.timestamp).localeCompare(String(b.timestamp));
      });
    }
  }
  return grouped;
}

function updateSelect(select, values, fallback) {
  select.innerHTML = "";
  for (const value of values) {
    const option = document.createElement("option");
    option.value = value;
    option.textContent = value;
    select.appendChild(option);
  }
  if (fallback && values.includes(fallback)) {
    select.value = fallback;
  }
}

function currentSelection() {
  return {
    slug: benchmarkSelect.value,
    runner: runnerSelect.value,
    caseName: caseSelect.value,
  };
}

function availableBenchmarks() {
  return Object.keys(state.grouped).sort();
}

function availableRunners(slug) {
  if (!state.grouped[slug]) {
    return [];
  }
  return Object.keys(state.grouped[slug]).sort();
}

function availableCases(slug, runner) {
  const reports = state.grouped[slug]?.[runner] || [];
  const cases = new Set();
  for (const report of reports) {
    Object.keys(report.entries || {}).forEach((name) => cases.add(name));
  }
  return Array.from(cases).sort();
}

function seriesForSelection(slug, runner, caseName) {
  const reports = state.grouped[slug]?.[runner] || [];
  const series = [];
  for (const report of reports) {
    const value = report.entries?.[caseName];
    if (typeof value !== "number") {
      continue;
    }
    series.push({
      label: report.timestamp || "",
      value,
      sha: report.git_sha || "",
    });
  }
  return series;
}

function renderChart(series, label) {
  const ctx = document.getElementById("benchmarkChart").getContext("2d");
  const labels = series.map((point) => point.label);
  const values = series.map((point) => point.value);

  if (!state.chart) {
    state.chart = new Chart(ctx, {
      type: "line",
      data: {
        labels,
        datasets: [
          {
            label,
            data: values,
            borderColor: "#c75b1c",
            backgroundColor: "rgba(199, 91, 28, 0.12)",
            borderWidth: 2,
            pointRadius: 4,
            pointBackgroundColor: "#1f6fb5",
            tension: 0.25,
          },
        ],
      },
      options: {
        responsive: true,
        plugins: {
          legend: {
            display: false,
          },
          tooltip: {
            callbacks: {
              label: (context) => {
                const value = formatSeconds(context.parsed.y);
                const sha = series[context.dataIndex]?.sha || "";
                return sha ? `${value} (${sha.slice(0, 7)})` : value;
              },
            },
          },
        },
        scales: {
          x: {
            ticks: {
              maxRotation: 45,
              minRotation: 0,
              color: "#6c5f54",
            },
            grid: {
              display: false,
            },
          },
          y: {
            ticks: {
              color: "#6c5f54",
              callback: (value) => formatSeconds(value),
            },
            grid: {
              color: "rgba(32, 27, 22, 0.08)",
            },
          },
        },
      },
    });
  } else {
    state.chart.data.labels = labels;
    state.chart.data.datasets[0].data = values;
    state.chart.update();
  }
}

function updateStats(series) {
  if (!series.length) {
    latestValue.textContent = "--";
    latestMeta.textContent = "No data";
    deltaValue.textContent = "--";
    deltaMeta.textContent = "";
    return;
  }
  const latest = series[series.length - 1];
  latestValue.textContent = formatSeconds(latest.value);
  latestMeta.textContent = latest.sha ? `sha ${latest.sha.slice(0, 7)}` : "";

  if (series.length > 1) {
    const previous = series[series.length - 2];
    const delta = ((latest.value - previous.value) / previous.value) * 100;
    deltaValue.textContent = formatPercent(delta);
    deltaMeta.textContent = `vs ${previous.sha ? previous.sha.slice(0, 7) : "previous"}`;
  } else {
    deltaValue.textContent = "--";
    deltaMeta.textContent = "";
  }
}

function refresh() {
  const { slug, runner, caseName } = currentSelection();
  const series = seriesForSelection(slug, runner, caseName);
  emptyState.textContent = series.length ? "" : "No data for selection.";
  renderChart(series, `${slug} / ${caseName}`);
  updateStats(series);
}

function initSelectors() {
  const benchmarks = availableBenchmarks();
  updateSelect(benchmarkSelect, benchmarks, benchmarks[0]);

  const runners = availableRunners(benchmarkSelect.value);
  updateSelect(runnerSelect, runners, runners[0]);

  const cases = availableCases(benchmarkSelect.value, runnerSelect.value);
  updateSelect(caseSelect, cases, cases[0]);

  benchmarkSelect.addEventListener("change", () => {
    updateSelect(runnerSelect, availableRunners(benchmarkSelect.value), runnerSelect.value);
    updateSelect(
      caseSelect,
      availableCases(benchmarkSelect.value, runnerSelect.value),
      caseSelect.value
    );
    refresh();
  });

  runnerSelect.addEventListener("change", () => {
    updateSelect(
      caseSelect,
      availableCases(benchmarkSelect.value, runnerSelect.value),
      caseSelect.value
    );
    refresh();
  });

  caseSelect.addEventListener("change", refresh);
}

async function bootstrap() {
  try {
    const response = await fetch("data.json", { cache: "no-store" });
    if (!response.ok) {
      throw new Error("Failed to load data.json");
    }
    const data = await response.json();
    state.data = data;
    state.grouped = groupReports(data.reports || []);

    updatedAt.textContent = data.generated_at
      ? new Date(data.generated_at).toLocaleString()
      : "--";
    reportCount.textContent = String((data.reports || []).length);

    if (!Object.keys(state.grouped).length) {
      emptyState.textContent = "No reports found.";
      return;
    }

    initSelectors();
    refresh();
  } catch (error) {
    emptyState.textContent = "Failed to load benchmark data.";
    console.error(error);
  }
}

bootstrap();
