# CellGen — Generativ designmotor for robotceller

## Prosjektoversikt

CellGen er et verktøy for robotintegratører som automatisk genererer og evaluerer robotcellelayout. Brukeren manipulerer constraints (cellestørrelse, innmatingspunkt, syklustidkrav, produktdimensjoner), og systemet genererer i sanntid robotplassering, bevegelser og cellekomponenter.

Inspirasjonen er spillet Tiny Glade — brukeren interagerer direkte med cellen, og systemet reagerer umiddelbart med oppdatert layout og animerte robotbevegelser. Alt skal føles responsivt og visuelt polert.

### Formål

Dette er en demo som skal vises til robotintegratører for å starte et forskningssamarbeid. Den trenger ikke være perfekt — den trenger å være overbevisende. Én applikasjon (palletering), gjort bra.

## Teknologistack

| Komponent | Teknologi | Rolle |
|-----------|-----------|-------|
| Visualisering & robotikk | **threepp** (C++20) | 3D-rendering, URDF-lasting, kinematikk, baneplanlegging, ROS2 |
| Fysikksimulering | **NVIDIA Newton** (Python/Warp) | Kollisjonsdeteksjon, dynamikk, energiestimering |
| Optimalisering | **Python** | Design space exploration, kandidatgenerering og evaluering |
| Kommunikasjon | **ROS2** | Grensesnitt mellom threepp og Newton |

### Viktig kontekst om threepp

Threepp er en C++20-port av three.js (r129) utviklet av en prosjektmedarbeider. Den har upubliserte robotikk-moduler inkludert URDF-loader, baneplanlegger og ROS2-støtte. Utvikleren er tilgjengelig for spørsmål og tilpasninger. Behandle threepp som primærbiblioteket — all visualisering og robotikkinteraksjon går gjennom det.

Repository: https://github.com/markaren/threepp

## Arkitektur

```
┌─────────────────────────────────────────────────┐
│                  Python orkestrering             │
│  (design space exploration, optimalisering)      │
│  - Genererer kandidatlayout                      │
│  - Evaluerer og rangerer løsninger               │
│  - Styrer parametrisk søk                        │
└──────────┬──────────────────┬────────────────────┘
           │ ROS2             │ ROS2
           ▼                  ▼
┌─────────────────┐  ┌────────────────────────────┐
│  NVIDIA Newton  │  │        threepp (C++)        │
│  - Kollisjon    │  │  - 3D-visualisering         │
│  - Dynamikk     │  │  - URDF robotmodeller       │
│  - Energi       │  │  - Kinematikk/baneplan      │
│                 │  │  - Brukerinteraksjon (UI)    │
└─────────────────┘  └────────────────────────────┘
```

## Demo-scope: Palleterings-celle

### Hva brukeren kan gjøre

- Dra i celledimensjonene (bredde, dybde)
- Flytte transportbåndets innmatingspunkt
- Flytte palleposisjon(er)
- Endre kartongstørrelse (bredde, dybde, høyde)
- Justere ønsket syklustid (enheter/minutt)
- Legge til/fjerne palleposisjoner

### Hva systemet genererer automatisk

- Robotmodell og plassering (valgt fra katalog basert på rekkevidde og payload)
- Monteringstype (gulv, sokkel — velges automatisk)
- Pallemønster (lagvis stabling optimalisert for stabilitet og utnyttelse)
- Animert pick-place-syklus
- Estimert syklustid
- Robotens arbeidsområde (visualisert som transparent sone)
- Sikkerhetssoner (minimum avstand til gjerder, basert på ISO 10218-2)

### Visuell kvalitet

Demoen skal se polert ut — dette selger konseptet:
- God belysning med skygger (threepp støtter dette)
- Subtilt grid på gulvet
- Fargekodet arbeidsområde (grønn = ok, gul = grenseverdi, rød = utenfor rekkevidde)
- Smooth animasjoner på robotbevegelser
- Snap-feedback når brukeren endrer parametere
- Enkel, ren UI — ikke overfylt med kontroller

## Komponentkatalog (asset-system)

Hver komponent beskrives i YAML med sine tillatte dimensjoner:

```yaml
# Tre typer dimensjonsconstraints:

# 1. Diskrete — faste størrelser, snap til nærmeste gyldige
safety_fence_panel:
  type: discrete
  variants:
    - { width: 1200, height: 1400, mesh: "fence_1200x1400.stl" }
    - { width: 1400, height: 1400, mesh: "fence_1400x1400.stl" }
    - { width: 1600, height: 2000, mesh: "fence_1600x2000.stl" }

# 2. Kontinuerlige — justerbare innenfor intervall med steglengde
conveyor_belt:
  type: continuous
  length: { min: 500, max: 6000, step: 50 }
  width: { options: [400, 600, 800] }
  mesh_strategy: tile  # repeter seksjoner

# 3. Sammensatte — satt sammen av standardmoduler
fence_side:
  type: composite
  components: safety_fence_panel
  strategy: fill  # fyll med standardpaneler + ett tilpasset
```

For demoen: hardkod 3-4 komponenter. Design datamodellen riktig for fremtidig utvidelse.

## Filformat

Bruk STEP for geometri (universelt støttet av robotikkverktøy) og JSON for cellebeskrivelsen:

```json
{
  "cell": {
    "dimensions": { "width": 4000, "depth": 3000 },
    "components": [
      {
        "type": "robot",
        "model": "UR10e",
        "position": { "x": 1500, "y": 1200, "z": 0 },
        "mount": "pedestal_500mm"
      },
      {
        "type": "conveyor",
        "length": 2500,
        "width": 600,
        "position": { "x": 0, "y": 600, "z": 850 }
      }
    ],
    "product": {
      "dimensions": { "width": 300, "depth": 200, "height": 150 },
      "weight": 5.0
    },
    "requirements": {
      "cycle_rate": 12,
      "pallet_pattern": "auto"
    }
  }
}
```

## Algoritmer som trengs

### 1. Pallemønster-generering
Gitt kartongdimensjoner og palledimensjoner (EUR-pall 1200x800), finn optimal stablingsplan per lag. Standard tilnærming: prøv 0° og 90° rotasjon per lag, maksimer arealutnyttelse, vurder overlapping mellom lag for stabilitet.

### 2. Robot-valg og plassering
Gitt påkrevd rekkevidde (avstand fra robot til pick-punkt og place-punkt) og payload (produktvekt + griper), velg minste robot fra katalog som oppfyller kravene. Plasser roboten slik at alle pick- og place-punkter er innenfor arbeidsområdet med margin.

### 3. Syklustid-estimering
Beregn tid per pick-place-syklus: tilnærmingsbevegelse + grip + løft + transport + plassering + retur. Bruk trapesprofil for hastighet (akselerasjon, maks hastighet, deselerasjon) basert på robotens spesifikasjoner.

### 4. Reachability-sjekk
For hvert pick/place-punkt: sjekk om punktet er innenfor robotens arbeidsvolum (sfærisk tilnærming minus indre radius). Flagg punkter som er marginale eller utenfor rekkevidde.

### 5. Sikkerhetsavstands-beregning
Basert på ISO 13855: sikkerhetsavstand = (robotens stoppetid × maks hastighet) + margin. Plasser gjerder minimum denne avstanden fra robotens ytterste rekkevidde.

## Prosjektstruktur

```
cellgen/
├── CLAUDE.md                    # Denne filen
├── CMakeLists.txt               # Hovedbyggsystem
├── src/
│   ├── core/                    # Kjernelogikk (C++)
│   │   ├── cell.hpp/cpp         # Celledatamodell
│   │   ├── component.hpp/cpp    # Komponentkatalog og constraints
│   │   ├── robot_catalog.hpp/cpp # Robotbibliotek med spesifikasjoner
│   │   └── pallet_pattern.hpp/cpp # Pallemønster-algoritme
│   ├── evaluation/              # Evaluering (C++)
│   │   ├── reachability.hpp/cpp # Reachability-analyse
│   │   ├── cycle_time.hpp/cpp   # Syklustidberegning
│   │   └── safety_zone.hpp/cpp  # Sikkerhetsavstandsberegning
│   ├── visualization/           # Threepp-basert (C++)
│   │   ├── cell_renderer.hpp/cpp    # Hovedvisualisering
│   │   ├── robot_animator.hpp/cpp   # Robotanimasjon
│   │   ├── interaction.hpp/cpp      # Brukerinteraksjon (dra, resize)
│   │   ├── zone_overlay.hpp/cpp     # Arbeidsområde/sikkerhetssone-visualisering
│   │   └── ui_panel.hpp/cpp         # Parameterkontroller
│   └── main.cpp                 # Applikasjons entry point
├── python/
│   ├── optimizer/               # Design space exploration
│   │   ├── generator.py         # Kandidatgenerering
│   │   ├── evaluator.py         # Evaluering via Newton
│   │   └── search.py            # Søkealgortime (GA/bayesiansk)
│   └── ros2_bridge/             # ROS2 kommunikasjon
│       └── cell_publisher.py    # Publiser celletilstand
├── assets/
│   ├── robots/                  # URDF-filer for roboter
│   ├── components/              # 3D-modeller (STL)
│   └── catalog/                 # YAML komponentdefinisjoner
├── config/
│   └── robot_specs.yaml         # Robotspesifikasjoner (rekkevidde, payload, hastighet)
└── tests/
    ├── test_pallet_pattern.cpp
    ├── test_reachability.cpp
    └── test_cycle_time.cpp
```

## Robotkatalog (startdata for demo)

```yaml
robots:
  - model: "UR10e"
    reach_mm: 1300
    payload_kg: 12.5
    max_speed_deg_s: 120
    repeatability_mm: 0.05
    weight_kg: 33.5
    type: cobot

  - model: "ABB IRB 460"
    reach_mm: 2400
    payload_kg: 110
    max_speed_deg_s: 200
    repeatability_mm: 0.1
    weight_kg: 855
    type: industrial_palletizer

  - model: "KUKA KR 40 PA"
    reach_mm: 2091
    payload_kg: 40
    max_speed_deg_s: 170
    repeatability_mm: 0.05
    weight_kg: 610
    type: industrial_palletizer

  - model: "Fanuc M-410iC/110"
    reach_mm: 2403
    payload_kg: 110
    max_speed_deg_s: 190
    repeatability_mm: 0.3
    weight_kg: 1020
    type: industrial_palletizer
```

## Relevante standarder

- **ISO 10218-2:2025** — Sikkerhetskrav for robotceller (layout, risikovurdering)
- **ISO 13855** — Sikkerhetsavstander basert på tilnærmingshastighet
- **ISO/TS 15066** — Collaborative roboter (kraft/hastighetsgrenser)
- **ISO 12100** — Generell maskinrisikovurdering (metodikk)
- **IEC 60204-1** — Elektrisk utrustning av maskiner (kabelføring)

## Utviklingsstrategi

### Fase 1: Statisk visualisering (uke 1-2)
- Sett opp threepp-prosjekt med CMake
- Last inn en URDF-robot og vis den i en enkel scene
- Lag gulvgrid, grunnbelysning, kameranavigasjon
- Vis en statisk palleterings-celle med hardkodede posisjoner

### Fase 2: Interaksjon (uke 3-4)
- Implementer dra-for-å-resize på celledimensjoner
- Implementer snap-logikk for komponentplassering
- Koble celledimensjoner til automatisk robotplassering
- Vis arbeidsområde-overlay som oppdateres i sanntid

### Fase 3: Algoritmer (uke 4-5)
- Pallemønster-generering
- Syklustid-estimering
- Robot-valg basert på rekkevidde/payload
- Sikkerhetsavstands-beregning

### Fase 4: Animasjon og polering (uke 5-7)
- Animert pick-place-syklus
- Smooth overganger når parametere endres
- UI-panel med parameterkontroller
- Sikkerhetssone-visualisering

### Fase 5: Newton-integrasjon (uke 7-8)
- Koble Newton via ROS2 for kollisjonssjekk
- Energiestimering per syklus
- Sammenligning av kandidatløsninger

## Viktige designprinsipper

1. **Responsivitet over presisjon.** Demoen må føles umiddelbar. Bruk forenklede beregninger som oppdateres i sanntid, ikke presise simuleringer som tar sekunder.

2. **Visuell kvalitet selger.** God belysning, skygger, rene farger. En integratør som ser noe som ser profesjonelt ut tar konseptet seriøst.

3. **Én applikasjon, gjort bra.** Ikke prøv å generalisere til sveising eller maskinbetjening i demoen. Palletering alene.

4. **Arkitektur for fremtiden.** Selv om demoen er smal, design datamodeller og interfaces slik at nye applikasjoner kan legges til uten omskriving.

5. **Test algoritmene isolert.** Pallemønster, reachability og syklustid skal ha unit tests uavhengig av visualiseringen.
