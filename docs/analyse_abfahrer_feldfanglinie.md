# Analyse: Wegfahren vom Erntefahrzeug und Feldfanglinie

## Problemumfang

Nach dem Ueberladen faehrt der Abfahrer teilweise ohne sinnvollen Loesungspfad vom
Erntefahrzeug weg. Dabei treten drei sichtbare Symptome gemeinsam auf:

- Der Fahrer verlaesst die Position am Erntefahrzeug ohne geeigneten Freifahrpfad.
- Die Feldgrenze beziehungsweise Fanglinie wird nicht als bevorzugter Korridor genutzt.
- Das HUD zeigt bereits `Fahre zu Zielpunkt`, obwohl das Gespann noch auf dem Feld steht.

Mit "Fanglinie" ist in dieser Analyse die befahrbare Linie innerhalb der Feldgrenze
gemeint. Im AutoDrive-Code existiert kein eigenes Datenmodell mit diesem Namen.

## Ablauf nach dem Ueberladen

1. `EmptyHarvesterTask` beendet das Ueberladen.
2. Nur bei einer Courseplay-Tasche oder einer erkannten Ernterwende wird direkt
   rueckwaerts gefahren.
3. Sonst wartet der Abfahrer kurz und beendet den Task.
4. `CombineUnloaderMode:getTaskAfterUnload()` waehlt den Folgetask.
5. Ist der Abfahrer noch nicht voll genug zum Abladen, wird direkt
   `DriveToDestinationTask` zum ersten Marker erzeugt.

Der letzte Schritt erklaert die HUD-Anzeige. In diesem Zweig gibt es keine Pruefung,
ob Fahrzeug oder Gespann noch auf beziehungsweise nahe am Feld stehen. Damit wird
`ExitFieldTask` umgangen.

Relevante Stellen:

- `scripts/Tasks/EmptyHarvesterTask.lua`: Zustaende `STATE_UNLOADING_FINISHED`,
  `STATE_REVERSING` und `STATE_WAITING`
- `scripts/Modes/CombineUnloaderMode.lua`: `getTaskAfterUnload()`
- `scripts/Tasks/DriveToDestinationTask.lua`: HUD und Netzanfahrt

## Festgestellte Ursachen

### 1. Fehlender Freifahr-Zustand nach dem Ernter

Nach dem Ueberladen existiert kein allgemeiner Task "Erntefahrzeug sicher verlassen".
Rueckwaertsfahrt wird nur ausgefuehrt, wenn Courseplay eine Tasche meldet, der Ernter
wendet oder der Abfahrer bereits lange feststeckt. Im Normalfall folgt nach einer
Wartezeit sofort der naechste Modus-Task.

Folgen:

- Kein Vergleich von Vorwaerts-, Rueckwaerts- und seitlichen Fluchtwegen.
- Kein Sicherheitsabstand zum Rohr, Schneidwerk, Ernter und Anbaugeraeten.
- Ein nachfolgender Pathfinder startet aus einer geometrisch unguenstigen Lage.
- Hindernisse werden erkannt, aber nicht in ein gezieltes Befreiungsmanoever
  uebersetzt.

### 2. Falscher Task-Uebergang bei teilgefuelltem Abfahrer

`CombineUnloaderMode:getTaskAfterUnload()` behandelt nur den Fall `filledToUnload`
mit einer Feldpruefung. Im Gegenfall wird bei deaktiviertem `parkInField` direkt
`DriveToDestinationTask` gesetzt. Das ist fachlich falsch, wenn das Gespann noch im
Feldbereich steht.

Das HUD ist daher nicht die Ursache. Es zeigt den tatsaechlich aktiven, aber falsch
ausgewaehlten Task an.

### 3. Widerspruechliche Felderkennung an der Grenze

Mehrere Definitionen von "auf dem Feld" werden gemischt:

- `AutoDrive.checkIsOnField()` prueft nur einen Weltpunkt ueber
  `FSDensityMapUtil.getFieldDataAtWorldPosition()`.
- Der Pathfinder verlangt fuer `startIsOnField` zusaetzlich einen positiven
  `frontSensorField`. Dieser Sensor fordert Feldboden an allen Ecken seiner Box.
- An der Fanglinie kann der Fahrzeugursprung noch auf dem Feld liegen, waehrend eine
  Sensorecke bereits ausserhalb liegt. Dann wird `startIsOnField = false`.
- Liegt das Ziel ausserhalb des Feldes, wird `restrictToField` ebenfalls deaktiviert.

Genau an der Fanglinie ist die Erkennung dadurch am instabilsten. Der Pathfinder kann
die Feldbegrenzung verlieren, obwohl der groesste Teil des Gespanns noch auf dem Feld
steht.

### 4. Feldgrenze ist harte Sperre und wird spaeter komplett deaktiviert

Der Pathfinder kennt keine bevorzugte Fanglinie und keinen weichen Abstand zur
Feldkante. Er kennt nur Feldzelle oder Nicht-Feldzelle. Bei Fehlschlag folgen
Fallbacks, die erst einen schmalen Aussenbereich erlauben und danach die
Feldbegrenzung vollstaendig deaktivieren.

Damit kann eine schwierige Startlage dazu fuehren, dass gerade jene Begrenzung
ignoriert wird, die den Abfahrer kontrolliert aus dem Feld fuehren sollte.

### 5. Pathfinder plant nur Vorwaertsfahrt

Der freie Pathfinder erzeugt keine Hybrid-A*- oder Reeds-Shepp-Manoever mit
Vorwaerts-/Rueckwaertswechsel. Bestehende Rueckwaertsfunktionen liegen in Tasks und
`SpecialDrivingModule`; sie sind reaktiv und erzeugen keinen kollisionsbewerteten
Gesamtpfad.

Ein Hindernis direkt vor dem Gespann kann deshalb einen gueltigen Pfad verhindern,
obwohl wenige Meter Rueckwaertsfahrt eine gute Startposition schaffen wuerden.

## Verbesserungsvorschlag

### Phase 1: Eigener `LeaveHarvesterTask`

Nach jedem abgeschlossenen Ueberladen zuerst einen gemeinsamen Freifahr-Task
ausfuehren. Dieser muss unabhaengig von Fuellstand, Courseplay und spaeterem Ziel sein.

Aufgaben:

- Sicherheitszone um Erntefahrzeug und dessen Anbaugeraete bestimmen.
- Kandidaten vorwaerts, schraeg vorwaerts, rueckwaerts und schraeg rueckwaerts bilden.
- Gesamtes Gespann beruecksichtigen, nicht nur Zugfahrzeugursprung.
- Kandidaten per Kollisionsmasken und Swept-Volume-Pruefung bewerten.
- Rueckwaertskandidaten nur zulassen, wenn
  `trailerModule:canBeHandledInReverse()` dies erlaubt.
- Erst nach erreichtem Sicherheitsabstand an Feld-/Netzpfadplanung uebergeben.

Bewertung eines Kandidaten:

`Kosten = Weglaenge + Wendekosten + Rueckwaertsaufschlag + Fruchtschaden + Feldrandverletzung + Hindernisrisiko`

Rueckwaertsaufschlag haelt Vorwaertsfahrt bevorzugt. Ist Vorwaertsfahrt blockiert,
wird Rueckwaertsbewegung trotzdem erreichbar.

### Phase 2: Robuste Feldzustandsentscheidung

Eine zentrale Funktion fuer den Moduswechsel einfuehren, zum Beispiel
`isVehicleTrainOnOrNearField(vehicle)`.

Sie sollte mehrere Punkte des Gespanns pruefen:

- Zugfahrzeugmitte
- Vorder- und Hinterachse
- Trailerachsen beziehungsweise Trailerenden
- konfigurierbare Toleranzzone beidseits der Feldgrenze

Mit Hysterese arbeiten: "Feld verlassen" erst bestaetigen, wenn das gesamte Gespann
fuer mehrere Updates ausserhalb liegt. Eine einzelne Sensorecke darf keinen
Task-Wechsel ausloesen.

`getTaskAfterUnload()` muss diese Pruefung in beiden Fuellstandszweigen verwenden.
Solange das Gespann auf oder nahe am Feld steht, muss HUD/Task `Verlasse Feld` statt
`Fahre zu Zielpunkt` anzeigen.

### Phase 3: Fanglinienkorridor statt binaerer Feldsperre

Aus Feldabfragen lokal eine innere Kontur abtasten. Abstand sollte mindestens aus
halber Gespannbreite plus Sicherheitsmarge bestehen. Kandidaten auf dieser Kontur
bilden einen Fanglinienkorridor.

Planungsreihenfolge:

1. Freifahren vom Ernter.
2. Kuerzester kollisionsfreier Weg zur inneren Fanglinie.
3. Entlang Fanglinie zu geeignetem Netzeinstieg beziehungsweise Feldausgang.
4. Erst danach normales AutoDrive-Netz verwenden.

Feldverletzung sollte zunaechst Kostenaufschlag statt harter Sperre sein. Vollstaendige
Deaktivierung der Grenze nur als letzter Fallback, wenn kein Korridor erreichbar ist.

Falls Courseplay eine verlaessliche Feldpolygon-/Kursgrenzen-API bereitstellt, kann
diese bevorzugt werden. Ohne Courseplay muss AutoDrive aus
`FSDensityMapUtil.getFieldDataAtWorldPosition()` eine lokale Kontur ableiten.

### Phase 4: Begrenzte Rueckwaertsplanung

Kein sofortiger kompletter bidirektionaler Pathfinder noetig. Zuerst begrenzte
Escape-Planung vor dem normalen Pathfinder:

- maximal etwa eine Gespannlaenge oder 20 bis 30 m
- wenige diskrete Lenkwinkel
- Kollisionspruefung fuer gesamte Fahrzeughuelle
- Abbruch bei Knickwinkelgrenze, Feldrandrisiko oder blockiertem Heck
- anschliessende Neuberechnung des Vorwaertspfads

Spaeter kann dies durch Hybrid A* mit Vorwaerts-/Rueckwaertszustaenden ersetzt werden.
Reeds-Shepp allein reicht fuer Gespanne nicht, weil Trailerknickwinkel und mehrere
Gelenke fehlen.

## Empfohlene Umsetzungsreihenfolge

1. Task-Uebergang in `getTaskAfterUnload()` korrigieren.
2. Zentralen Feldstatus fuer gesamtes Gespann mit Hysterese einfuehren.
3. `LeaveHarvesterTask` mit sicheren Vorwaerts-/Rueckwaerts-Escape-Kandidaten bauen.
4. Fanglinienkorridor erzeugen und als Zwischenziel verwenden.
5. Pathfinder-Fallbacks auf weiche Feldrandkosten umstellen.
6. HUD um klare Phasen `Entferne mich vom Ernter`, `Verlasse Feld` und
   `Berechne Netzanfahrt` ergaenzen.

## Testszenarien

- Abfahrer halbvoll und voll nach stationaerem Ernter entlassen.
- Abfahrer exakt auf, knapp innerhalb und knapp ausserhalb der Feldgrenze.
- Ernter auf Fanglinie, Abfahrer zwischen Ernter und Feldinnerem.
- Baum oder Fahrzeug vor dem Abfahrer, freier Raum rueckwaerts.
- Hindernis hinten, freier Vorwaertsweg.
- Einachs-, Drehschemel- und Sattelauflieger.
- Courseplay-Tasche, Ernterwende und normaler Stillstand.
- Feld mit engem Ausgang und Feld mit mehreren Netzeinstiegen.
- `restrictToField`, `avoidFruit`, `parkInField` und `followOnlyOnField` jeweils an/aus.
- Multiplayer: Task- und HUD-Zustand auf Server und Clients identisch.

## Fazit

Problem liegt nicht nur in Bubble-Auswahl. Vor Netzanfahrt fehlt eine eigene,
hindernisbewusste Freifahrphase. Gleichzeitig waehlt Modus bei teilgefuelltem Abfahrer
zu frueh `DriveToDestinationTask`, und Felderkennung verliert an Grenze ihre
Restriktion. Sinnvolle Loesung braucht daher Task-Korrektur, robuste Erkennung des
gesamten Gespanns, Fanglinien-Zwischenziele und begrenzte Rueckwaerts-Escape-Planung.
