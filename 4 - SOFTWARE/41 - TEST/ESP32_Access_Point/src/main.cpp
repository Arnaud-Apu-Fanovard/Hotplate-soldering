#include "WiFi.h"
#include <WebServer.h>


WebServer server(80);

// Fonction pour gérer la page principale
void handleRoot() {
  // Page HTML envoyée par l'ESP32
  String page = R"rawliteral(
    <!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Hot Plate Soldering</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            text-align: center;
            margin: 20px;
        }

        h1 {
            color: #333;
        }

        button {
            padding: 10px 20px;
            font-size: 16px;
            margin: 20px;
            cursor: pointer;
        }

        .input-group {
            display: inline-flex; /* Met les champs et unités sur la même ligne */
            align-items: center; /* Aligne verticalement le texte */
            margin: 20px;
        }

        input {
            padding: 10px;
            font-size: 16px;
            text-align: center;
            width: 150px;
            margin-right: 10px; /* Espacement entre le champ et l'unité */
        }

        span {
            font-size: 16px;
        }

        .control-panel {
            display: flex;
            justify-content: center;
            align-items: center;
            margin: 20px;
        }

        #liveTemperature {
            font-size: 24px;
            margin-left: 20px;
        }

        canvas {
            display: block;
            margin: 0 auto;
            width: 80%;
            height: 400px;
        }
    </style>
</head>
<body>

    <h1>Hot Plate Soldering</h1>

    <!-- Input pour la température de consigne avec unité sur la même ligne -->
    <div class="input-group">
        <input type="number" id="targetTemperature" placeholder="Température finale" min="0" max="300">
        <span>°C</span>
    </div>

    <!-- Input pour la rampe de température avec unité sur la même ligne -->
    <div class="input-group">
        <input type="number" id="rampRate" placeholder="Rampe de température" min="0" max="10" step="0.1">
        <span>°C/s</span>
    </div>

    <!-- Input pour régler l'échelle de temps de l'axe des abscisses -->
    <div class="input-group">
        <input type="number" id="timeScale" placeholder="Durée totale (s)" min="10" max="600" value="60">
        <span>secondes</span>
    </div>

    <!-- Panneau de contrôle avec le bouton ON/OFF et la température en direct -->
    <div class="control-panel">
        <!-- Bouton ON/OFF -->
        <button id="toggleButton">ON</button>

        <!-- Lecture en direct de la température -->
        <div id="liveTemperature">Température actuelle : 20°C</div>
    </div>

    <!-- Graphique -->
    <canvas id="temperatureChart"></canvas>

    <script src="https://cdn.jsdelivr.net/npm/chart.js"></script>
    <script>
        // Variables globales
        let targetTemperature = 0;
        let rampRate = 0;
        let currentConsigne = 0;
        let systemOn = false;
        let currentTemperature = 20; // Température initiale
        let timeScale = 60; // Durée totale affichée sur le graphique en secondes (échelle de temps)

        const targetInput = document.getElementById('targetTemperature');
        const rampInput = document.getElementById('rampRate');
        const timeScaleInput = document.getElementById('timeScale');
        const toggleButton = document.getElementById("toggleButton");
        const liveTemperatureDisplay = document.getElementById("liveTemperature");

        // Gestion du bouton ON/OFF
        toggleButton.addEventListener("click", function() {
            systemOn = !systemOn;
            toggleButton.textContent = systemOn ? "OFF" : "ON";
        });

        // Récupération de la consigne finale (température cible)
        targetInput.addEventListener('input', function() {
            targetTemperature = parseFloat(targetInput.value);
        });

        // Récupération de la rampe de température
        rampInput.addEventListener('input', function() {
            rampRate = parseFloat(rampInput.value);
        });

        // Récupération de l'échelle de temps
        timeScaleInput.addEventListener('input', function() {
            timeScale = parseInt(timeScaleInput.value);
            updateChartScale();
        });

        // Création du graphique avec Chart.js
        const ctx = document.getElementById('temperatureChart').getContext('2d');
        let temperatureData = Array(timeScale).fill(currentTemperature); // Initialiser avec la température actuelle
        let consigneData = Array(timeScale).fill(currentConsigne); // Consigne calculée

        const temperatureChart = new Chart(ctx, {
            type: 'line',
            data: {
                labels: Array.from({length: timeScale}, (_, i) => i), // Temps de 0 à timeScale secondes
                datasets: [{
                    label: 'Température actuelle (°C)',
                    data: temperatureData,
                    borderColor: 'rgba(255, 99, 132, 1)',
                    backgroundColor: 'rgba(255, 99, 132, 0.2)',
                    fill: true,
                    tension: 0.1
                }, {
                    label: 'Consigne actuelle (°C)',
                    data: consigneData,
                    borderColor: 'rgba(54, 162, 235, 1)',
                    borderDash: [10, 5], // Ligne pointillée
                    fill: false
                }]
            },
            options: {
                scales: {
                    x: {
                        title: {
                            display: true,
                            text: 'Temps (s)'
                        }
                    },
                    y: {
                        min: 0,
                        max: 300,
                        title: {
                            display: true,
                            text: 'Température (°C)'
                        }
                    }
                }
            }
        });

        // Fonction pour mettre à jour la consigne actuelle en fonction de la rampe
        function updateConsigne() {
            if (currentConsigne < targetTemperature) {
                currentConsigne += rampRate; // Incrémenter la consigne selon la rampe
                if (currentConsigne > targetTemperature) {
                    currentConsigne = targetTemperature; // Ne pas dépasser la consigne finale
                }
            }
            updateChart();
        }

        // Fonction pour simuler la température actuelle (tu peux remplacer cela par des données réelles via ESP)
        function simulateTemperature() {
            if (systemOn) {
                // Simule une montée de la température (tu peux remplacer cela par un vrai capteur)
                if (currentTemperature < currentConsigne) {
                    currentTemperature += rampRate / 2; // Simulation de l'évolution de la température
                    if (currentTemperature > currentConsigne) {
                        currentTemperature = currentConsigne;
                    }
                }
            }
            liveTemperatureDisplay.textContent = `Température actuelle : ${currentTemperature.toFixed(1)}°C`;
        }

        // Fonction pour mettre à jour l'échelle de temps du graphique
        function updateChartScale() {
            temperatureData = Array(timeScale).fill(currentTemperature); // Réinitialiser les données de température
            consigneData = Array(timeScale).fill(currentConsigne); // Réinitialiser les données de consigne
            temperatureChart.data.labels = Array.from({length: timeScale}, (_, i) => i); // Ajuster l'axe du temps
            temperatureChart.update(); // Actualiser le graphique
        }

        // Fonction pour mettre à jour le graphique
        function updateChart() {
            // Décaler les données de température et de consigne
            temperatureData.shift();
            temperatureData.push(currentTemperature);

            consigneData.shift();
            consigneData.push(currentConsigne);

            temperatureChart.update();
        }

        // Boucle de mise à jour du graphique toutes les secondes
        setInterval(function() {
            simulateTemperature();
            updateConsigne();
        }, 1000);

    </script>

</body>
</html>
  )rawliteral";
  
  server.send(200, "text/html", page);
}






void setup() {
  Serial.begin(115200);
  WiFi.softAP("ESP_AP", "123456789");
    Serial.print("Host Name:");
  Serial.println(WiFi.softAPgetHostname());
  Serial.print("Host IP:");
  Serial.println(WiFi.softAPIP());
  Serial.print("Host IPV6:");
#if ESP_ARDUINO_VERSION_MAJOR < 3
  Serial.println(WiFi.softAPIPv6());
#else
  Serial.println(WiFi.softAPlinkLocalIPv6());
#endif
  Serial.print("Host SSID:");
  Serial.println(WiFi.SSID());
  Serial.print("Host Broadcast IP:");
  Serial.println(WiFi.softAPBroadcastIP());
  Serial.print("Host mac Address:");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("Number of Host Connections:");
  Serial.println(WiFi.softAPgetStationNum());
  Serial.print("Host Network ID:");
  Serial.println(WiFi.softAPNetworkID());
  Serial.print("Host Status:");
  Serial.println(WiFi.status());
  Serial.println("");


  server.on("/", handleRoot);
  server.begin();
  
  Serial.println("Serveur web démarré");
}



void loop() {
  Serial.print("Number of Host Connections:");
  Serial.println(WiFi.softAPgetStationNum());
  server.handleClient();

}