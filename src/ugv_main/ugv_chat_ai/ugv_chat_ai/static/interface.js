const chatMessagesDiv = document.getElementById("chat-messages");
const userInputElem = document.getElementById("user-input");
//const modelToggle = document.getElementById("model-toggle");
//const modelLabelLeft = document.getElementById("model-label-left");
//const modelLabelRight = document.getElementById("model-label-right");
// State variables
//let modelName = modelToggle.checked ? "gpt-4" : "gpt-3.5-turbo";
let messages = [];
let systemMessageRef = null;
let autoScrollState = true;

// Event listener functions
function handleModelToggle() {
  if (modelToggle.checked) {
    modelLabelRight.textContent = "GPT-4";
    modelName = "gpt-4";
  } else {
    modelLabelLeft.textContent = "GPT-3.5";
    modelName = "gpt-3.5-turbo";
  }
}

function handleInputKeydown(event) {
  if (event.key === "Enter" && !event.shiftKey) {
    event.preventDefault();
    document.getElementById("submitBtn").click();
  }
}

function autoScroll() {
  if (autoScrollState) {
    chatMessagesDiv.scrollTop = chatMessagesDiv.scrollHeight;
  }
}

// Event listeners for functions above
//modelToggle.addEventListener("change", handleModelToggle);
document.getElementById("user-input").addEventListener("keydown", handleInputKeydown);

document;
chatMessagesDiv.addEventListener("scroll", function () {
  const isAtBottom =
    chatMessagesDiv.scrollHeight - chatMessagesDiv.clientHeight <=
    chatMessagesDiv.scrollTop + 1;

  autoScrollState = isAtBottom;
});

window.renderMarkdown = function (content) {
  const md = new markdownit();
  return md.render(content);
};

function highlightCode(element) {
  const codeElements = element.querySelectorAll("pre code");
  codeElements.forEach((codeElement) => {
    hljs.highlightElement(codeElement);
  });
}

function addMessageToDiv(role, content = "") {
  let messageDiv = document.createElement("div");
  messageDiv.className =
    role === "user" ? "message user-message" : "message assistant-message";

  let messageText = document.createElement("p");
  messageDiv.appendChild(messageText);

  if (content) {
    let renderedContent = window.renderMarkdown(content).trim();
    messageText.innerHTML = renderedContent;
    highlightCode(messageDiv);
  }

  chatMessagesDiv.appendChild(messageDiv);
  autoScroll();

  return messageText;
}
