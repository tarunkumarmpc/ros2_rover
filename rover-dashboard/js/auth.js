const USERS = [
  { username: "operator", password: "robotics123" },
  { username: "admin", password: "adminpass" },
  { username: "admin", password: "admin" }
];
function authenticate(username, password) {
  return USERS.some(user => user.username === username && user.password === password);
}
const loginPage = document.getElementById('login-page');
const dashboard = document.getElementById('dashboard');
const loginForm = document.getElementById('login-form');
const loginError = document.getElementById('login-error');
const userDisplay = document.getElementById('user-display');
loginForm.addEventListener('submit', function(e) {
  e.preventDefault();
  const username = document.getElementById('login-username').value.trim();
  const password = document.getElementById('login-password').value;
  if (authenticate(username, password)) {
    loginPage.style.display = "none";
    dashboard.style.display = "";
    userDisplay.textContent = username;
    initializeDashboard();
  } else {
    loginError.classList.remove("hidden");
  }
});

