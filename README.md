# SCARA Robot Controller – Analyse und Plan

Stand: 25.10.2026. Neues, zunächst lokales Projekt für die geplante Neufassung.
Es enthält noch KEIN neues Roboterprogramm: Der Benutzer möchte zuerst den Plan
besprechen und weitere eigene Probleme ergänzen.

- [Problemkatalog](docs/PROBLEMKATALOG.md): Benutzerbeobachtungen und Codebefunde.
- [Umsetzungsplan](docs/PLAN.md): Anforderungen, Architektur, Befehle, Abnahmekriterien.
- [Analyse und Reproduktion](docs/ANALYSE.md): geprüfter Stand, Belege, Grenzen.
- [Hardwarefragen](docs/HARDWAREFRAGEN.md): vor Hardwarefahrten zu klären.

## GitHub-Status

Vorgeschlagener Repositoryname: SCARA-Robot-Controller, zunächst privat. Name und
Sichtbarkeit wurden angefragt und sind nicht abschließend bestätigt.
Noch kein GitHub-Repository erstellt und noch nichts hochgeladen:
Git-Lesezugriff auf das alte Repository funktioniert, aber `gh api user` und der
Zugriff auf api.github.com werden in dieser Umgebung mit Forbidden blockiert.
Das beweist keinen fehlenden Git-Lesezugriff und keinen Bedarf an einem neuen
Token. Keine Geheimnisse in Chat oder Dateien eintragen.

Benötigt wird funktionierender GitHub-API-Zugriff mit Berechtigung, im gewünschten
Konto ein Repository zu erstellen. Alternativ kann der Benutzer ein leeres
Repository erstellen und dessen URL bereitstellen; Schreibzugriff ist danach
separat zu prüfen. Die genannte HowToMechatronics-Seite liefert ebenfalls einen
Proxy-CONNECT-403 und wurde deshalb noch nicht inhaltlich verifiziert.
