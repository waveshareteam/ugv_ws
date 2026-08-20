document.addEventListener("DOMContentLoaded", function () {
  if (typeof mermaid === "undefined") {
    return;
  }
  mermaid.initialize({
    startOnLoad: false,
    theme: "default",
    securityLevel: "loose",
  });
  // Mermaid 10+: explicitly render div.mermaid from superfences fence_div_format
  if (typeof mermaid.run === "function") {
    mermaid.run({ querySelector: ".mermaid" });
  } else {
    mermaid.init(undefined, document.querySelectorAll(".mermaid"));
  }
});
