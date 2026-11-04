# BIOBUZZ Competition Manual (TU07) - Full Notes

Generated 2026-11-03 from ~/Downloads/BIOBUZZ_Manual.pdf (171 pages). Tags: [CODE] = affects robot software/hardware, [STRATEGY], [DERIVED] = computed from drawings, verify against CAD, [AMBIGUOUS].

# BIOBUZZ Competition Manual (TU07) - Notes, Pages 1-45

Source: /tmp/biobuzz_out/manual.txt (PDFKit extraction), with pages 21, 25, 27, 29, 36, 42-44 checked visually against the PDF.
Legend: **[CODE]** = relevant to robot software; **[STRATEGY]** = relevant to match/event strategy; **[AMBIG]** = ambiguous or worth confirming.

Blank pages: 4, 24, 30, 40 (verified by rendering, no content).

---

## Page 1 - Cover
- "2026-2027 FIRST Tech Challenge Competition Manual, BIOBUZZ(TM) Presented by RTX, Version: TU07."

## Pages 2-3 - Table of Contents (171 pages total)
| Sec | Title | Page |
|---|---|---|
| 1 | Introduction (1.1-1.7) | 5 |
| 2 | FIRST Season Overview | 19 |
| 3 | Competition Eligibility and Inspection (I): 3.1 Team Eligibility 20; 3.2 Awards Eligibility 21; 3.3 MATCH Eligibility 21 | 20 |
| 4 | Advancement: 4.1 Points Calc 26; 4.2 Distribution by Region 28 | 25 |
| 5 | Event Rules (E): 5.1 General 31; 5.2 Machine Shops 34; 5.3 Wireless 35; 5.4 Load-In 36; 5.5 Pits 36; 5.6 Carts 38; 5.7 Ceremonies 39 | 31 |
| 6 | Awards (A): 6.1 Overview/Schedule 41; 6.2 Rules 46; 6.3 Descriptions 51; 6.4 Tournament ALLIANCE Awards 55; 6.5 Individual 56; 6.6 Project-Based Global 57 | 41 |
| 7 | Game Sponsor Recognition | 59 |
| 8 | Game Overview | 60 |
| 9 | ARENA: 9.1 Dimensions 61; 9.2 FIELD 62; 9.3 Areas/Zones 63; 9.4 TILE Coordinates 65; 9.5 ALLIANCE AREA 66; 9.6 HIVE 67; 9.7 FLOWER 70; 9.8 SCORING ELEMENTS 72; 9.9 AprilTags 72; 9.10 FIELD STAFF 76; 9.11 EMS 77 | 61 |
| 10 | Game Details: 10.1 MATCH Overview 79; 10.2 DRIVE TEAM 79; 10.3 Setup 80; 10.4 MATCH Periods 83; 10.5 Scoring 84; 10.6 Violations 90; 10.7 Head REFEREE 95; 10.8 Other Logistics 95 | 79 |
| 11 | Game Rules (G): 11.1 Safety 97; 11.2 Conduct 97; 11.3 Pre-MATCH 100; 11.4 In-MATCH 104 | 97 |
| 12 | ROBOT Construction (R): 12.1 General 118; 12.2 Fair Play 122; 12.3 Fabrication 124; 12.4 SIGN 126; 12.5 Motors & Actuators 129; 12.6 Power 132; 12.7 Control/Command/Signals 136; 12.8 Pneumatics 141; 12.9 OPERATOR CONSOLE 141 | 116 |
| 13 | Tournament (T): 13.1 143; 13.2 Replays 143; 13.3 Question Box 144; 13.4 General 145; 13.5 Practice 149; 13.6 Qualification 149; 13.7 Playoff 152; 13.8 Dual Division 159 | 143 |
| 14 | League Play Tournaments (L) | 163 |
| 15 | FIRST Championship (C): 15.1 Awards Mods; 15.2 Game Mod; 15.3 3-ROBOT ALLIANCES; 15.4 Pit Crews 166; 15.5 Playoffs 166 | 165 |
| 16 | Glossary | 167 |

Useful pointers for software: 9.4 TILE Coordinates (p65), 9.6 HIVE (p67), 9.9 AprilTags (p72), 10.4 MATCH Periods (p83), 10.5 Scoring (p84), 12.7 Control System (p136). Note 15.3: FIRST Championship uses 3-ROBOT ALLIANCES.

---

## 1 Introduction (pp. 5-18)

### 1.1 About FIRST (p5)
- FIRST = "For Inspiration and Recognition of Science and Technology."
- Purpose: prepare young people of today for the world of tomorrow. Vision: transform culture so science/technology are celebrated. Mission: life-changing robotics programs giving young people skills, confidence, and resilience to build a better world.

### 1.2 FIRST Tech Challenge (pp5-6)
- Differentiators: Scale, Skill Level, Complexity, Flexibility, Student Ownership.
- Classroom-sized field, classroom-safe robots; can be built from a Starter Kit, fully custom, or a mix; non-classroom tools optional.
- Season: kicks off in **September**, runs through **March**; capstone events **April through July**.

### 1.3 FIRST Ethos and Core Values (pp6-8)
- 1.3.1 Gracious Professionalism (GP): coined by Dr. Woodie Flowers (1943-2019). Aspirational ideal, not a measuring stick; never say someone "is"/"is not" GP. Talk about specific behaviors instead. (See E102.B: "weaponizing GP" is a violation.)
- 1.3.2 Coopertition: kindness and respect amid fierce competition; today's opponent may be the next match's alliance partner.
- 1.3.3 Core Values: Discovery, Innovation, Impact, Inclusion, Teamwork, Fun.
- 1.3.4 Why embrace it: build a better world, pursue a meaningful life.

### 1.4 The Spirit of the Competition (pp9-12)
- 1.4.1 Note from FIRST staff: "There are no perfect solutions, only tradeoffs."
  - **Key 2026-27 philosophy change:** the manual emphasizes the **"spirit of the rule"** and "intended gameplay" and empowers volunteers to make **good-faith judgment calls**, instead of trying to cover every scenario. Volunteers watch for larger behaviors and strategies that go against the spirit (the example given is "strategically blocking an opponent from access to their gate vs. going into an opponent's gate zone"). The manual says this approach will continue in future seasons.
  - **[STRATEGY]** Expect referees to penalize strategies that go against the intent, not just literal rule violations. Don't build strategy around loopholes or technicalities. Note the "gate" reference: blocking an opponent's gate access is the kind of behavior refs are told to watch (see Sec 10/11 for the actual gate rules).
- 1.4.2 Framework of Behaviors: GP & Core Values; FIRST Code of Conduct (adults and parents of minors certify); Competition Integrity Contract (mandatory for every team); Competition Rules (Manual, Team Updates, Q&A).
- 1.4.3 Role of Mentors: mentor involvement should be "however much is needed to inspire the youth on the team"; no prescriptive process. (The extracted text shows a stray "1.5 CIC" heading inside this section. That's a layout artifact.)
- 1.4.4 Spirit of FIRST Volunteers: volunteers should actively help teams avoid breaking rules; "Giving Back" and "Paying It Forward"; Program Delivery Partner (PDP) coordinates volunteers.

### 1.5 Competition Integrity Contract (CIC) (pp13-15)
Two parts. The Sporting Ethics Code is mandatory. The Behavior Guidelines are aspirational.
- **1.5.1 Sporting Ethics Code**, "Our Team Certifies":
  - **We Don't Cheat**: never intentionally violate the spirit of a rule; don't circumvent FIRST systems.
  - **We Always Behave with Integrity**: follow all regulations, including ones that aren't checked.
  - **We Play the Game as Intended**: "Red Alliance vs. Blue Alliance," not "Teams vs. Volunteers." **No exploiting "loopholes."** When wording is ambiguous, follow the intent of the rule. **[STRATEGY]**
  - **We Know Our Team Doesn't Enforce the Rules**: report concerns politely to staff. Don't accuse, investigate, or publicly call anyone out.
- **1.5.2 Behavior Guidelines**: Journey before Destination; Respect & Good Sportsmanship ("We support our teammates. We praise our partners. We celebrate our opponents."); Help Each Other; Take the High Road; Respect Property and Facilities (report accidental damage); Positive Culture of Safety; Healthy Choices (hydration, food, activity, sleep, breaks); We're All on the Same Team.
- **1.5.3 Infractions, Mitigations, & Escalation**: CIC violations can happen at or outside events. Regulation-specific escalations follow the "Escalation Guidelines" (separate doc). Escalation can go to LRI, Head Referee, or FTA. Serious violations, especially of the Sporting Ethics Code, go to the Event Director and/or FIRST HQ. Possible outcomes: sanction, disqualification from current/future events, suspension, or permanent removal from FTC. Open door: customerservice@firstinspires.org.

### 1.6 Accessibility and Inclusion (p15)
- Reasonable accommodations on request. Contact local leadership **before** the event. Local leadership may make rule exceptions as long as they don't create undue hardship or safety concerns.

### 1.7 Understanding & Using the Competition Manual (pp15-18)
- 1.7.1 Conventions:
  - The Competition Manual is the **"source of truth"**. It takes precedence over supporting docs when they disagree.
  - Unreleased cross-references appear as [G###] etc.
  - ALL CAPS words are defined terms (Sec 16 Glossary).
  - Rule letter prefixes: **I** = Sec 3, **E** = Sec 5, **A** = Sec 6, **G** = Sec 11, **R** = Sec 12, **T** = Sec 13, **L** = Sec 14, **C** = Sec 15. Next digit(s) = subsection, final digits = position (e.g., E301 = Sec 5, subsection 3, rule 01) (Figure 1-3).
  - Orange boxes (warnings/cautions/notes) are part of the manual but **do not carry rule weight**. The rule wins any conflict.
  - **Imperial units are authoritative**. Metric conversions (rounded to the nearest 0.05 cm, e.g., "17.5 in. (~44.45 cm)") are for reference only. **[CODE]** Use imperial (inches) as the canonical unit in field/pose constants, which matches our 144x144-inch field convention.
  - Headlines: **Evergreen rules** have bold green headlines with a leading asterisk (*), recur every season, and come first in their section. Other headlines are bold orange. If the headline disagrees with the rule text, the rule text wins (report the disparity to customerservice@firstinspires.org).
- 1.7.2 Translations: the English PDF on the Game and Season Materials page is authoritative. The FTC AI Chatbot is not authoritative. Text-based English version is available only for assistive devices.
- 1.7.3 **Team Updates**: posted **every Thursday**, starting on Kickoff day and ending **two weeks before FIRST Championship**. Additions are highlighted in yellow; deletions are struck through. **Team Updates published after an event's driver's meeting do not apply to that event.** **[STRATEGY]** Check Team Updates every Thursday. Rule changes could affect code (e.g., scoring, AUTO rules).
- 1.7.4 **Q&A System**:
  - Opens **September 28, 2026, 12:00 p.m. ET**. Only accessible through **Lead Coach 1 or Lead Coach 2's** FIRST dashboard account.
  - Moderators start answering questions each **Monday**. The window closes **Thursday 5:00 p.m. ET**.
  - Q&A answers **do not supersede** the manual. **REFEREES and INSPECTORS are the ultimate authority** at events.
  - Questions that won't be answered: rulings on vague situations, challenges to past event decisions, legality design reviews, overly broad questions with no rule references, how a ref should have ruled at a past event, duplicates, questions already clearly answered in the manual.
  - Good questions are generic and cite specific rules.

---

## 2 FIRST Season Overview (p19)
- Season theme: **FIRST CANOPY(TM)** (2026-2027), "Engineer a Thriving Planet with FIRST", nature- and biodiversity-inspired. https://www.firstinspires.org/first-canopy

---

## 3 Competition Eligibility and Inspection (I) (pp20-23)

### 3.1 Team Eligibility Rules (p20)
- **I101 \*Teams must be registered with FIRST.** A team must be "competition ready" to compete in official events, earn MATCH points, or be eligible for judged awards.
  - A. North America: (i) complete annual registration on the FIRST dashboard; (ii) pay the annual fee; (iii) **2 adults** in the Lead Coach 1/Lead Coach 2 roles who have passed **Youth Protection Program (YPP)** screening; (iv) any additional regional YPP requirements; (v) register all youth members on the dashboard.
  - B. Outside North America: (i) registration on the dashboard; (ii) any local PDP requirements (fees, registration, YPP).
  - Note: the PDP may, rarely, allow delayed payment case by case. **Unpaid teams earn no official season record and cannot advance.**
- **I102 \*Check-in at the event on time.** An **adult** team member must check in at Pit Administration (or the designated location) **no later than 45 minutes before Qualification MATCHES are scheduled to start**, unless the Event Director specifies or approves otherwise. **At least one STUDENT must be at the venue** before check-in can be completed. Possible regional requirements: A. current roster from the dashboard; B. PDP registration/consent forms; C. a ROBOT built to Sec 12 specs (if playing MATCHES); D. a printed PORTFOLIO (optional; see Sec 6).
  - Note: inspection or judging may happen before the check-in deadline. Check the schedule. **[STRATEGY]**
- **I103 \*A responsible adult must be present for the whole event.** At least **1, preferably 2**, responsible adults present at all times. Recommended to be on the roster.

### 3.2 Awards Eligibility Rules (p21)
- Team Judged Award eligibility **requires participation in an Initial Interview (A203)**. Some awards also require a **PORTFOLIO (A201)**.
- Teams at multiple Qualifying or League Tournaments, or at events outside their HOME REGION, are subject to **A213 and A214** (in Sec 6.2, beyond this range).

### 3.3 MATCH Eligibility Rules (pp21-23)
- Teams must **pass ROBOT inspection** before Qualification or Playoff MATCHES.
- Note: teams **may** play **scheduled** Practice MATCHES before passing inspection, but **may not** play unscheduled/"filler line" Practice MATCHES before passing. The FTA, LRI, or Head REFEREE may bar an unsafe robot from Practice MATCHES.
- **3.3.1 Inspection**: students present the ROBOT, all COMPONENTS, and the OPERATOR CONSOLE. The **LRI has final authority** on eligibility in matters of ROBOT construction. Teams may power on and enable the robot during inspection to demonstrate compliance. Tell inspectors about stored energy (e.g., stretched springs).
  - **[CODE]** Have a simple "inspection demo" OpMode or mode ready to show mechanisms moving and the robot's full range of configurations/extension safely on demand (see I301).
  - Inspection is not comprehensive. **STRATEGICALLY** circumventing construction rules violates the CIC. Use the **Inspection Checklist** to self-inspect before every event.
  - Inspection can be split across breaks, multiple inspectors, and the LRI on request.
- **I301 \*Bring the complete ROBOT and supporting equipment to inspection.** Present all COMPONENTS (including decorative ones) and the OPERATOR CONSOLE. Not every inspected component has to be used in every MATCH, but **don't add uninspected COMPONENTS without re-inspection**. **Demonstrate every way the ROBOT will be configured for a MATCH.**
- **I302 \*Certain limits apply to all configurations.** The **total** number of electronics (motors, servos, Android devices, etc.) across **all MECHANISMS and the base ROBOT**, whether or not they're used at the same time, may not exceed Sec 12 limits. **[CODE]** The motor and servo budget is the union across all swappable configurations. Check the Sec 12.5 counts against our hardware map (dual flywheel, turret servo, intake, launch gate, drive, indicator-light servos all count).
- **3.3.2 Re-Inspection**: most changes need re-inspection before further Qualification/Playoff MATCHES. The Head REF or FTA can request it. Deliberately changing the robot after inspection for an advantage without re-inspecting goes against the spirit.
- **I303 \*Request re-inspection for most changes to the ROBOT.** Generally **not** needed (unless the change significantly affects size, legality, or safety) for:
  - A. adding, moving, or removing fasteners (cable ties, tape, rivets)
  - B. labeling or marking
  - C. adding, moving, or replacing the team SIGN
  - **D. revising ROBOT code** **[CODE]** Code changes between matches don't need re-inspection.
  - E. swapping a COMPONENT for an identical one
  - F. swapping a MECHANISM for an identical one (same size, weight, material)
  - G. adding, removing, or reconfiguring with a subset of already-inspected MECHANISMS
  - Note: if unsure (e.g., whether reassembly changes size), check with the LRI or FIELD STAFF.
  - **[AMBIG]** D has the general caveat: a code change that alters robot size, legality, or safety (e.g., new extension limits or a starting-configuration change) arguably needs re-inspection. Ask the LRI if code changes expansion or starting size.
- **I304 \*Do not exploit re-inspection.** It can't be used to get around other rules.

---

## 4 Advancement (pp25-29)

### General (pp25-26)
- **Teams can only advance from events in their HOME REGION.** They may be invited to out-of-region tournaments but **cannot advance from them**.
- HOME REGION is shown on the FTC-Events team page (Figure 4-1: "Region:" field). Teams without a local PDP, or geographically isolated within their region, can email customerservice@firstinspires.org to be reassigned to a more accessible region **once per season** for advancement.
- **Teams can advance from any of their first three entry-level events** (Qualifying Tournaments (QT) and League Tournaments (LT)). Teams may play in more than 3, but only the **first three chronological events** count for advancement. **[STRATEGY]** Choose and sequence events carefully. Events after the third can't qualify us.
- **Only one League per season.** All teams in a League play in its League Tournament.
- **Figure 4-2 Tournament Advancement Structure** (verified visually):
  - League Meets 1, 2, 3 (LM) -> League Tournament (LT)
  - Qualifying Tournament (QT) and LT -> Super Qualifying Tournament (SQT, optional) -> Regional Championship (RCMP) -> FIRST Championship (FCMP) and/or FIRST Premier Event (FPE)
  - QT/LT can also go directly to RCMP.
  - Pilot event -> FCMP / FPE (Pilot: only in select new or emerging regions)
  - "Non-Advancement" events are a separate box (optional).
  - Footnotes: 1, 3, 5 = optional, not offered in all regions (League Meet 3, SQT, Non-Advancement); 2 = all teams in a League play in the LT; 4 = RCMP may be called State, Region, or Country Championship; 6 = Pilot only in select new/emerging regions.
- **A team may only participate in one SQT.**
- The local PDP sets advancement numbers for each tournament up to the RCMP. FIRST staff set advancement from RCMP to FCMP and FPEs.

### 4.1 Advancement Points Calculation (pp26-28)
At each advancing event, teams are ranked by advancement points. The top-ranked teams **not already advanced** fill that event's allocated slots.

**Table 4-1: Advancement Point Assignment**
| Category | Points |
|---|---|
| Qualification Phase Performance | Normal-distributed **16 (top) down to 2** (min 2, max 16) using the 4.1.1 equation |
| ALLIANCE lead | **21 minus ALLIANCE lead number** (e.g., ALLIANCE #3 lead = 18) |
| Draft Order Acceptance | **21 minus Draft Order Acceptance number** (e.g., accepting 3rd draft position = 18) |
| Playoff Advancement | **40** Winners (1st), **20** Finalists (2nd), **10** 3rd, **5** 4th (Sec 13.8 modifies for Dual Division) |
| Team Judged Awards | **Inspire: 60 / 30 / 15** (1st/2nd/3rd); **other awards: 12 / 6 / 3** (1st/2nd/3rd); see A211 for eligible awards |

**Table 4-2: Advancement Sorting Criteria / Tiebreakers** (in order)
1. Total Advancement Points
2. Judged Team Award Points
3. Playoff Advancement Points
4. ALLIANCE Selection Results Points (Lead or Draft Order Acceptance)
5. Qualification Phase Performance Points
6. **Average Qualification MATCH Points (excluding FOULS)**
7. **Average Qualification AUTO Points**
8. Highest individual Qualification MATCH Points (excluding FOULS)
9. Second-highest individual Qualification MATCH Points (excluding FOULS)
10. Random selection by the Event Management System

**[STRATEGY]** Points earned from fouls the opponent commits against us do not count in tiebreakers 6, 8, or 9. Average AUTO points is tiebreaker 7, so a strong, consistent autonomous helps even in advancement tiebreaks. **[CODE]** This is another reason to prioritize AUTO reliability.

**4.1.1 Qualification Phase Performance** (formula verified visually, p27):

    QualificationPoints(R, N, a) = ceil( InvERF( (N - 2R + 2) / (a*N) ) * ( 7 / InvERF(1/a) ) + 9 )

- R = qualification rank at the end of Qualification MATCHES (per EMS, defined in 13.6.3)
- N = number of FTC teams in the Qualification rounds at the event
- a (alpha) = **1.07** (static)
- Brackets are a **ceiling**.
- Table 4-3 (28-team example): Rank 1 = 16, 2 = 15, 3 = 14, 4 = 14, ..., 12 = 10, 13 = 10, 14 = 10, ..., 25 = 6, 26 = 5, 27 = 5, 28 = 4.
- My recomputation matches Table 4-3 exactly. Other event sizes, computed with the formula (rank 1 to last):
  - N=16: 16,15,13,13,12,11,11,10,9,9,8,8,7,6,6,4
  - N=24: 16,15,14,13,13,12,12,11,11,11,10,10,9,9,9,8,8,8,7,7,6,6,5,4
  - N=32: 16,15,15,14,13,13,13,12,12,12,11,11,11,10,10,10,9,9,9,9,8,8,8,7,7,7,6,6,6,5,4,4
- **[AMBIG]** Table 4-1 says the range is "16 to 2," but in practice the last-place team gets more than 2 at typical event sizes: 4 at N=28, 3 at N=40-64. The minimum approaches 2 only as N grows large. The maximum is always 16.
- **[STRATEGY]** The gaps between ranks are small (about 1 point per few ranks) compared with Playoff wins (40/20/10/5) and awards (Inspire 60; others 12/6/3). Alliance selection is also worth up to 20 points (21-1). Judged awards and playoff results dominate advancement.

**4.1.2 ALLIANCE Selection Results**: ALLIANCE leads get the same points as the team drafted in the same sequence position. Example: the team accepting the 3rd ALLIANCE lead's pick gets the same points as the 3rd lead (18). Rationale: leads are about as strong as equivalently drafted teams.
- **[AMBIG]** "Draft Order Acceptance number" is defined by sequence. The text's example (3rd draft position = 18) implies that in a multi-round draft (e.g., FCMP 3-robot alliances), later picks could get very low or negative values. Check the Sec 13.7 / 15.3 alliance selection details.

**4.1.3 Playoff Performance**: all teams in an ALLIANCE get the Table 4-1 points for how far they advance. Bracket details are in 13.7.2.

**4.1.4 Team Judged Awards**: points apply only to team awards judged **at the event**, limited by A215 (max awards per team). Non-judged awards, individual awards (e.g., FIRST Leadership Award), or awards not judged at the event (e.g., Safety Animation Award) earn **no points**. Points for awards not given are not reassigned. List of eligible awards: A211.

### 4.2 Advancement Distribution by Region (pp28-29)
- The PDP sets advancement within the region. Minimum advancement numbers should be public **as early as possible before the event, and no later than when ALLIANCE selection begins**. They may be shown on FTC-Events (Figure 4-3 example: "8 teams advance to NYC SUPER QUALIFIER 2").
- FIRST HQ allocates FCMP and FPE slots based on: global and regional representation; new developing regions with a PDP; **number of teams registered in the region before the cutoff date, November 17, 2026** (minimum registration requirements and total team count).
- Regional allocations are published on FTC-Events **starting early December**. Unsecured slots go back to FIRST HQ or the Premier Event Host for reallocation (possibly to a new region or a waitlist team).

---

## 5 Event Rules (E) (pp31-39)
Scope: from the start of the public schedule (team arrival) through departure.

**Universal Violation Note:** any E-rule violation = **VERBAL WARNING**. Egregious or repeated violations can be escalated to FIRST HQ and/or lead to **disqualification from MATCHES and awards**. Criminal behavior = removal of the individuals and/or team DQ from the event. Unsafe or out-of-spec items must be removed. PDPs may add stricter local or venue rules (e.g., badging, accessibility seating) and should communicate them early.

### 5.1 General Rules (pp31-34)
- **E101 \*Personal safety comes first.** A. Safety glasses or safety-rated glasses with side shields (ANSI-approved, UL Listed, CE EN166, AS/NZS certified, or CSA rated) in specified areas, with a **10-minute grace period** when the venue opens each day. B. Closed-toe/heel shoes. C. Control entanglement: tie back long hair, remove lanyards, spirit wear, and rings as needed near robots/tools. D. Appropriate clothing. E. Walk. F. Follow government and venue health rules (e.g., masks). Note: clear or lightly tinted glasses preferred; shaded allowed without an accommodation request; teams bring their own PPE; see the FIRST Safety Manual.
- **E102 \*Be Respectful.** Follow the Sec 1.4 framework; no uncivil behavior. Inappropriate: A. offensive language; B. **"weaponizing Gracious Professionalism"** (accusing someone of being "not GP"); C. blocking views for an extended period (momentarily holding up team signs is fine); D. anything that interferes with a ROBOT or ARENA operations. Possible ejection: E. assault (including unintentionally hitting someone with a thrown object); F. threats; G. harassment (badgering after a decision); H. bullying; I. insulting; J. swearing at a person; K. yelling at people in anger.
- **E103 \*Children with adults, please.** Children **under 12** must be accompanied by an adult in the pits at all times.
- **E104 \*Respect the venue.** No damage; no littering, including giveaways (candy, flyers, stickers) or items thrown from the stands.
- **E105 \*Event resources are for competing teams only.** Only registered teams may use the competition field, practice field, and inspection, unless the ED/PDP pre-approves. Host teams supplying practice field or machine shop resources may use them, but registered teams get priority.
- **E106 \*Practice only when/where permitted.** Robots may only run in the team's pit, designated practice areas, or Practice MATCHES. No practice setups outside the pit. The ED can shut down unsafe or interfering pit practice. Demonstrating the robot to guests or JUDGES is not practice. **[STRATEGY]** Plan pit testing (e.g., flywheel spin-up or turret tests) so it's safe and contained.
- **E107 \*Work in designated areas only.** FABRICATED ITEMS may only be made: A. in your pit; B. in another team's pit with permission; C. while queued for a MATCH or practice field (extra safety scrutiny); D. in staff-designated areas (e.g., playoff pit); E. at machine shops available to all teams.
- **E108 \*Some things do not belong at events.** No: A. skateboards; B. hoverboards; C. drones; D. bottled gas tanks (e.g., helium); E. noisemakers (floor stompers, whistles, air horns); F. scooters (except for accommodations); **G. any item with bright lights that flash faster than approximately 5 times per second.**
  - **[CODE] [AMBIG]** E108.G targets items brought to the event. Robot lighting is likely governed by Sec 12, but to be safe, keep any blinking pattern on our servo-PWM indicator lights at **5 Hz or slower** (e.g., "ready to launch" blink) until Sec 12 is checked.
- **E109 \*Do not arrange for additional services or utilities.** No arranging power, internet, or phone lines with venue providers. Don't use venue internet reserved for the event (EMS, webcasting).
- **E110 \*Do not sell stuff.** No sales (raffles, food, apparel, candy, drinks, promotional items) unless the ED allows it.
- **E111 \*No Loud Music.** No live bands, no loud music.
- **E112 \*Hang banners with care.** A. Don't cover or move other team or sponsor signs. B. Share space. C. Don't block spectators. D. ED permission needed outside your pit. E. Hang safely. F. **Banners outside pits must be 25 sq ft (~2.3 m^2) or smaller.** Remove everything (tape, string) at the end.
- **E113 \*Limit flag and flagpole size if used in the ARENA.** Guideline: flags **< 3 ft x 5 ft** (~91 x 152 cm) and **< 2 lbs** (~907 g). Flagpoles **8 ft or shorter** (~243 cm) and **< 3 lbs** (~1360 g).
- **E114 \*No firearms or other weapons**, including realistic props. Doesn't apply to law enforcement or venue security.
- **E115 \*Inspection required for practice FIELD access.** Only robots that have passed an initial, complete inspection may use the practice field. **Applies only at events without scheduled inspection times.** **[STRATEGY]** Get inspected early so practice field time is available for tuning.
- **E116 \*Do not record anyone at the event without their consent.** Volunteers may walk away if recorded without consent. Livestream consent doesn't cover recording specific interactions. Laws vary, and it may be criminal.
- **E117 \*Enter only 1 ROBOT in the tournament.** Each team inspects and plays MATCHES with **one ROBOT per event**, and may participate in **only one concurrent event**. Other robots may be brought for displays or awards.
- **E118 \*No saving seats.** No banners or ribbons to reserve seats. The ED may reserve seats for accessibility, volunteers, or playoff teams.

### 5.2 Machine Shops and Host Team Build Spaces (pp34-35)
- Rare. Run during set hours in the public agenda, usually sponsored by the host. All competing teams should have equal access.

### 5.3 Wireless Rules (p35)
- **E301 \*No wireless communication.** Teams may not set up their own **Wi-Fi (802.11a/b/g/n/ac/ax/be)** (access points or ad-hoc networks), **Bluetooth**, or any other 2.4 GHz or 5 GHz communications in the venue.
  - Notes: a hotspot from a phone, camera, smart TV, etc. counts as an access point. Disable smart TV APs (often on by default). Bluetooth (2.4 GHz) can interfere with venue and ROBOT systems. Don't operate R/C toys, drones, or FPV gear.
  - **[CODE]** Implications for us:
    - Panels dashboard and laptop tuning over the Control Hub's own Wi-Fi: the robot's own network is presumably governed by Sec 12.7 (R rules). Confirm whether connecting a laptop to the robot's network in the pits is allowed.
    - **No Bluetooth gamepads**, Bluetooth keyboards/mice, or phone hotspots for ADB-over-Wi-Fi at the venue.
    - Limelight configuration over its own AP or Wi-Fi isn't allowed. Configure it over USB/Ethernet or before the event.
    - **[AMBIG]** E301 and Sec 12 need to be read together for the Robot Controller/Driver Station Wi-Fi Direct link.
- **E302 \*Don't interfere with wireless networks.** Don't interfere with, or try to connect to, any other team's or FIRST's network without permission. Report vulnerabilities to the FTA/ED, or customerservice@firstinspires.org after the event. **[CODE]** Don't connect dev laptops to other teams' robot networks or the FIRST field network.

### 5.4 Load-In (p36)
- Large or multi-day events may publish load-in windows before pits open. Be safe, gracious, professional.

### 5.5 Pits (pp36-38)
- Team pit is typically **no larger than 10 ft x 10 ft x 10 ft** (~3 m x 3 m x 3 m) and marked with the team number. Size can vary by venue.
- The "pit area" includes aisles, pit admin, inspection, practice field, and anywhere robots are active. Pit rules apply to the whole area.
- Note: the ED may add limitations, communicated **at least 48 hours before event start** and applied fairly.
- A table and outlet aren't guaranteed. If there are no individual outlets, the venue must provide team-usable outlets in the pit area for battery charging. **Power may not be available overnight at multi-day events.** **[STRATEGY]** Bring enough charged batteries.
- **E501 \*Pits are unavailable if closed.** No access outside designated hours.
- **E502 \*Pit setups self-contained within the assigned space.** May not: A. run power or internet lines elsewhere (unless the ED allows); B. swap pits when pits have assigned team numbers; C. move to empty pits without ED approval.
- **E503 \*Keep aisles and exit pathways clear.**
- **E504 \*Open flames and spark-emitting devices limited and carefully controlled.** Generally OK: a lighter to melt string or strap ends; a sander shortening a bolt with sparks directed safely, briefly. Prohibited: welders, bench and angle grinders, gas torches.
- **E505 \*Only small benchtop machinery in pits.** Small means easily lifted by one person (3D printers, small band saws, small drill presses, desktop CNC mills, sanders). No floor-standing power tools (full-size drill presses or band saws).
- **E506 \*No brazing or welding.**
- **E507 \*Solder with an electric iron/gun only.**
- **E508 \*Pit structures must be safe.** No structures that support people or store items overhead. Don't block fire sprinklers. Popup tents are OK if they comply with E502 and don't interfere with fire suppression.
- **E509 \*Aerosols and chemicals with noxious fumes only in approved areas.** Some venues ban them entirely.
- **E510 \*Don't heat or cool ROBOT components to gain an advantage.** Robots should compete as if they'd been at venue ambient temperature for a long time. Violations: cooling fuses, heating batteries, freezing heatsinks. Allowed: heat shrink, batteries slightly warm from normal charging, components warm from prior use. **[STRATEGY]** Don't pre-warm batteries for flywheel performance. **[CODE]** Flywheel feedforward tuning should be robust to normal battery and temperature variation (consider voltage compensation).
- **E511 \*Charge batteries in a safe and fair manner.** A. Safe rate, following manufacturer recommendations. B. **Never use a charger exceeding a 3-amp average channel current.** C. Safe connectors only: polarized, matching the battery's connector. **No alligator clips.**

### 5.6 ROBOT Carts (p38)
- **E601 \*Carts must not be a nuisance.** Must: A. be easy to control and maneuver; B. pose no risk to bystanders; C. **fit through a standard 30-inch (762 mm) door**; D. stay in the pit when not in use; E. have no music or sound devices; F. **not use powered propulsion**.

### 5.7 Ceremonies (p39)
- Opening and closing ceremonies. Closing ceremony elements are worked into Playoff MATCHES. Trophies and medallions are presented at awards.
- **E701 \*Quiet in the pits during ceremonies** (outside of Playoff MATCHES): no A. power tools; B. loud hand tools (hammers, saws); C. disruptive shouting or yelling.
- **E702 \*Pit person limit during ceremonies is 5.** Max **5 team members** in the pits during ceremonies outside of Playoffs. **At least 1 representative must watch the ceremonies** to relay information.
- **E703 \*Be respectful during anthems.** Peaceful behavior. Abstaining is allowed if silent and non-disruptive.

---

## 6 Awards (A) (pp41-45; continues beyond this range)

### Intro (p41)
- Different event types and sizes offer different awards. **No awards at League Meets.**
- Roles: **JUDGES** meet teams, evaluate against criteria, and pick winners as a group with JA guidance. **JUDGE Advisor (JA)** trains, directs, and supervises judges and oversees process compliance, but **does not select winners**.
- Formats: traditional in-person judging, or **hybrid** (in-person gameplay with some or all judging done remotely before gameplay). Same standards.
- Recommended reading: Judge and Judge Advisor Manuals; **Outreach Terms and Definitions Document**.

### 6.1 Team Judged Awards Overview and Schedule (pp41-42)
- Three categories: **Machine, Creativity, and Innovation (MCI)**; **Team Attributes (TA)**; **Documentation**. Plus **Judges' Choice** (for a standout team not recognized in other categories). **Inspire Award** is the top all-around award.
- PDP-specific local or sponsor awards are **not** Team Judged Awards for advancement.
- **Figure 6-1 Award hierarchy** (verified visually):
  - Inspire Award spans MCI + TA + Think.
  - **MCI:** Design Award, **Control Award**, Innovate Award.
  - **TA:** Connect Award, Reach Award, Sustain Award.
  - **Documentation:** Think Award.
  - Judges' Choice Award is separate.
- Definitions: Inspire = excels in MCI, TA, and Documentation. MCI = technical accomplishments in brainstorming, design, construction, **operation, and control** of robots. TA = expanded skills, sustainability plan, outreach. Think = masterful documentation of process and robot in the PORTFOLIO. Judges' Choice = unique efforts, performance, or dynamics that don't fit elsewhere.
- **[CODE] [STRATEGY]** The **Control Award** (MCI) is where our software work counts: odometry fusion with Limelight relocalization, the shot-solution and readiness gating, the turret auto-aim at the nearest HIVE CELL with hysteresis, Pedro Pathing autos, and flywheel PID+FF tuning. Document these in the PORTFOLIO (criteria in Sec 6.3, beyond this range).
- Note: winners are "positive examples of the award criteria, not necessarily the 'best' team." Judges consider only the published Sec 6.3 criteria.
- **Teams may be judged regardless of inspection status, and are eligible for awards even without a ROBOT.**

### 6.1.1 Sources of Information Considered for Awards (p43)
- Sources: **Initial Interview**, **follow-up interview(s)**, **PORTFOLIO** (where applicable) (Figure 6-2).
- Judges only consider information from the **current event and current season**.
- **For judging, the current season began January 1, 2026.** Per A201.E, progress since **January 1, 2026** can be documented in the PORTFOLIO. **[AMBIG]** This predates the September 2026 kickoff. It appears intentional (it allows off-season work since Jan 1, 2026), but note it.
- Not considered: past performance, personal knowledge of a team, websites or social media, **ROBOT match performance (unless an award criterion)**, ROBOT penalties, tournament ranking.

### 6.1.2 Initial Interview (pp44-45)
- Formats: **Scheduled in-person** (set time, dedicated space); **Unscheduled in-person** (judges come to the pit or a dedicated space); **Remote** (video conference). Same rules for all. All teams at an event use the same format, set by the ED with the JA and regional planning committee and communicated before the event.
- **Figure 6-3 timeline:** 1. Judges introduce themselves. 2. **Team presentation, 5 minutes or less**, uninterrupted, "up to about 5 minutes, per A205". 3. **Q&A, 5 minutes or more** (open-ended questions for the rest of the time). 4. Interview concludes. 5. Judges discuss and fill out the **feedback form**. 6. Next team.
- Note: pit interviews can be noisy. For pit interviews, judges coordinate with queuers and technical volunteers so teams can still make MATCHES and work on robots.
- Review the **Judge Interview Question Bank**. At each event, the JA picks **two questions from the bank that every team is asked first**: **one MCI-focused** and **one TA-focused**. Judges may then ask more questions, including some not in the bank. **[STRATEGY]** Prepare software and control talking points for the MCI question.

### 6.1.3 Pit Interview(s) (p45)
- After Initial Interviews, judges may do informal follow-up pit interviews. Teams can share additional material (robot prototypes, design artifacts, outreach photos and letters). No new presentation needed.
- Note: judges may read additional material during pit interviews but **won't take content back** to deliberations. **[STRATEGY]** Have a laptop or tablet ready to show code, telemetry graphs, and tuning data (e.g., Panels plots).

### 6.1.4 Sustained Outreach and Demonstrating Impact by Numbers (p45)
- Sustained, ongoing outreach is valued over one-off events. Judges look at impact on the people reached.
- Use the Outreach Terms and Definitions Document for terms like starting a FIRST team, running an event, or reaching X people. Judges may probe those terms. (Section continues on p46+.)

---

## Cross-references to rules outside this range (to verify in other notes)
- A201 (PORTFOLIO, incl. A201.E), A203 (Initial Interview required), A205 (5-minute presentation), A211 (points-eligible awards), A213/A214 (multi-event / out-of-region award limits), A215 (max awards per team).
- 13.6.3 Qualification Ranking (rank R), 13.7.2 Playoff bracket, 13.8 Dual Division point modifications, Sec 14 League rules, 15.3 3-ROBOT ALLIANCES.
- Sec 12.5 motor/servo limits (I302), Sec 12.7 control system and wireless (with E301).
- "Escalation Guidelines" (separate doc), Inspection Checklist, Judge Interview Question Bank, Outreach Terms and Definitions Document.

## Key dates and deadlines (from this range)
| Date / cadence | Item |
|---|---|
| Sep 2026 | Kickoff. Team Updates start (every Thursday) |
| **Sep 28, 2026, 12:00 p.m. ET** | Q&A opens (Lead Coach 1/2 accounts only) |
| Weekly: Monday open to Thursday 5:00 p.m. ET | Q&A answering window |
| **Nov 17, 2026** | Regional registration cutoff for FCMP/FPE slot allocation (2 weeks from today, Nov 3, 2026) |
| Early Dec 2026 | Regional advancement allocations published on FTC-Events |
| 2 weeks before FCMP | Last Team Update |
| Jan 1, 2026 | Start of "current season" for judging/PORTFOLIO content |
| Each event: 45 min before Quals | Adult check-in deadline (with 1+ student present) |
| Each event day: first 10 min after the venue opens | Safety-glasses grace period |
| 48 h before event | Deadline for ED to announce extra pit limitations |

---

# BIOBUZZ Competition Manual (TU07) - Notes, Pages 46-95

Source: /Users/rahuloh/Downloads/BIOBUZZ_Manual.pdf, pages 46-95. Every page was read: text came from manual.txt, and every figure page was rendered and viewed. Pages 58 and 78 are intentionally blank.

Legend: **[CODE]** = matters for robot software (auto paths, targeting, vision). **[STRATEGY]** = matters for game strategy. **[AMBIGUOUS]** = unclear or conflicting in the manual. **[DERIVED]** = a value I computed from manual dimensions; the manual does not state it directly.

---

## Coordinate frame used in these notes

- **Origin**: the field corner at the **red alliance wall / audience wall** intersection, on the inside wall surfaces, at TILE top surface.
- **+X**: from the red ALLIANCE wall (x=0) toward the blue ALLIANCE wall (x=144).
- **+Y**: from the audience wall (y=0) toward the rear (far) wall (y=144).
- **+Z**: up from the TILE top.
- Units are inches. The field is 144 x 144.
- This matches the convention in `TeamCode/.../common/LaunchTargets.java`: red is on low X, and Y runs from audience 0 to far 144.
- In the manual's top-view figures (9-2, 9-4, 9-5, 9-17, 10-2) the audience is at the bottom and red is on the left, so figure-left = -X and figure-up = +Y.
- TILE seams fall at multiples of 24 in. TILE columns A-F run along X: A = 0-24 ... F = 120-144. TILE rows 1-6 run along Y: 1 = 0-24 (audience) ... 6 = 120-144 (rear). Seam-line names (Fig 9-4) are V,W,X,Y,Z at x = 24,48,72,96,120 and 1..5 at y = 24..120.

---

## Section 6 Awards (p46-57), from rule A201 onward (the earlier part of Section 6 is in the previous range)

### 6.2 Team Judged Award Rules
- **A201** PORTFOLIO limits:
  - 1 cover page with the team number. Optional on the cover: name, TOC, organizations, sponsors, logo, motto, robot/team photo.
  - **No more than 15 pages of content**, on US Letter (8.5x11 in) or A4 (210x297 mm).
  - A digital submission must be **< 15 MB**.
  - Content only from **on or after January 1, 2026**.
  - Judges do not use the cover page to evaluate. Pages past 15 are not read.
  - Minimize PII: first names and last initials only. Student photos are OK; full names are not.
  - Recommended: fonts of 10 pt or larger, good contrast. Judges will not click links or watch videos. Extra printed papers are not taken back to the judging room.
  - AI and research aids are allowed with a footnote/endnote credit (e.g., "Portfolio created by Team XXXXX and ChatGPT").
  - You may reference previous seasons, but the emphasis must be on the current season.
- **A202** Submit as the Event Director instructs and by the deadline. With no other instructions, bring **1 printed copy to the Initial Interview**. Keeping a spare copy in the pit is encouraged.
- **A203** An Initial Interview is required to be eligible for any judged award.
- **A204** Bring to the Initial Interview:
  - at least 2 STUDENTS (for teams of 2 or more),
  - the PORTFOLIO,
  - optional show-and-tell items (robot encouraged but optional),
  - optionally 1 silent observer (A208),
  - optionally 1 accommodation support person (A209).
  - A team without a robot can still be judged. Robot demos are allowed but must not cause significant delays.
- **A205** Every team gets the same interview length, **at least 10 minutes**.
- **A206** The timer starts after the judges introduce themselves and the team begins presenting or Q&A begins. Slow-starting teams are warned, and then the timer starts anyway.
- **A207** The first **5 minutes** are an uninterrupted prepared presentation, if the team wants it. The team may end it early. The rest is Q&A led by the judges.
- **A208** One adult silent observer is allowed. Coaches may be present outside the Initial Interview but may not interact with or coach students during any judge interaction.
- **A209** A translator or sign-language interpreter is allowed, usually team-provided. The team can request **2-5 extra minutes** in advance. The interpreter can be in addition to the silent observer.
- **A210** No photos, video, or audio recording during the Initial Interview (in addition to E116).
- **A211 / Table 6-1**: the number of awards scales with the number of participating teams (4-10 / 11-20 / 21-40 / 41-64).
  - Inspire: 1st / 1st-2nd / 1st-3rd / 1st-3rd.
  - Think: 1st / 1st / 1st-2nd / 1st-2nd (3rd discretionary).
  - At 4-10 teams, only one of Connect/Reach/Sustain is given, and only one of Innovate/Control/Design.
  - In larger brackets these awards add 2nd place (some discretionary, marked *) and 3rd place (discretionary).
  - Judges' Choice is always optional.
  - Only the awards in Table 6-1 count for advancement points.
  - Dual-division events: see 13.8.
- **A212** All teams get feedback from the Initial Interview, based on the first impression only. It is returned with the portfolio, near the end of the event, or digitally through FTC-Scoring to Lead Coach 1.
- **A213** Inspire is awarded only in the team's **HOME REGION**. Exceptions: FIRST Championship and Premier Events.
- **A214** A team can win 1st Inspire only once per season at Qualifying/League Tournaments. It can still win 2nd/3rd later, and 1st again at a Super-Qualifier or Regional Championship.
- **A215** One judged award (win or runner-up) per team per event. Alliance and individual awards are separate.

### 6.3 Award Descriptions (criteria)
- **Inspire (6.3.1)**: PORTFOLIO required. The team must be a strong contender in each of: MCI awards, TA awards, and Think. It must be positive and inclusive, and able to share its experience.
- **Think (6.3.2)**: PORTFOLIO required. It must include at least one of: (A) the engineering process, (B) lessons learned in robot design, (C) comparison of design choices, (D) math used for robot or programming decisions. It should be easy to read.
- **Connect (6.3.3)**: no PORTFOLIO required. Required: a professional-development plan with learning goals and steps. Encouraged: STEM networking and collaboration.
- **Reach (6.3.4)**: no PORTFOLIO required. Required: outreach objectives, strategy, and how they grow FIRST, plus recruitment of new teams, coaches, mentors, or volunteers. Encouraged: ambassadorship and media materials.
- **Sustain (6.3.5)**: no PORTFOLIO required. Required: long-term plan (financial, season, team) and how progress is tracked. Encouraged: leadership development and risk management.
- **Innovate (6.3.6, sponsored by RTX)**: no PORTFOLIO required. Engineering evidence, a creative or unique design, reliable most of the time. Encouraged: explain how risk was reduced.
- **Control (6.3.7)**: **PORTFOLIO required.** The PORTFOLIO must include the hardware/software control COMPONENTS, the challenges each solves, and the function of each. The robot must use hardware/software solutions with **external feedback** to control and improve performance. Encouraged: consistent across matches, reliability explained or demonstrated, lessons learned. Full code is not needed.
  - **[CODE][STRATEGY]** Our Pinpoint odometry + Limelight relocalization + flywheel PID/FF + turret auto-aim fits this award well. Document it in a summary.
- **Design (6.3.8)**: no PORTFOLIO required. The robot must be elegant, efficient, or maintainable, and the whole-robot design is considered. Encouraged: appearance, design rationale, consistency.
- **Judges' Choice (6.3.9)**: optional, for teams that don't fit the other categories.
- **6.4** Winning Alliance and Finalist Alliance awards. Dual or multi-division events give them at the division and finals level.
- **6.5.1** FIRST Leadership Award (see its webpage).
- **6.5.2** Compass Award: optional at Regional Championship level and available at Championship. It honors a coach or mentor, through a student-made video of **40-60 s** (max 60 s including credits). Formats: .mp4/.mov/.avi/.wmv, no streaming links, one per team per event, licensed music credited.
- **6.6.1** Digital Animation Award (sponsored by WPI): project-based and global. It does not count toward advancement. It is also open to FRC teams.

## Section 7 Game Sponsor Recognition (p59)
- The presenting sponsor is RTX.

---

## Section 8 Game Overview (p60)

- 2 ALLIANCES of 2 teams collect **POLLEN** (yellow) and **NECTAR** (alliance-colored). They **LAUNCH** them into their alliance's **HIVE** and **place** them into **FLOWERS**.
- **AUTO is the first 30 s.** Robots can use sensors to collect POLLEN from FLOWERS and LAUNCH it into their HIVE's CELLS. When enough elements are LAUNCHED into a CELL, the HIVE **tips**. Each TIP earns points and **unlocks additional NECTAR**.
- **TELEOP is the remaining 2:00.** Robots keep scoring and tipping.
- **Endgame**: alliances fill FLOWERS with POLLEN and NECTAR.
  - Points for the **bottom-most NECTAR** of your color in each FLOWER (the "first in" bonus, 10.5.2).
  - Points for **owning** a FLOWER, which goes to the alliance whose NECTAR is the **top-most**.
- RANKING POINTS come from additional achievements.

---

## Section 9 ARENA (p61-77)

### 9 intro / Fig 9-1 (p61)
- The ARENA is the FIELD, SCORING ELEMENTS, queue area, team media area, and event equipment.
- Fig 9-1 is an isometric view, labeled as follows:
  - the HIVE (red and blue) on a central Frame;
  - FLOWERS on each of the 4 walls;
  - LOADING ZONE-RED in the far corner on the red side; LOADING ZONE-BLUE on the audience end of the blue side;
  - GARDEN-RED on the audience wall at the red corner; GARDEN-BLUE on the rear wall at the blue corner;
  - ALLIANCE AREAS on the left (red) and right (blue) of the audience view.

### 9.1 Dimensions and Accuracy (p61-62)
- The **3D CAD model is the official field representation**. Measurements from it carry a general tolerance of **+/- 1 in (2.5 cm)**.
- Other references: the Event Field Setup Guide (build instructions and key dimensions), the Field Acceptance Checklist (controlled dimensions with tolerances, functional tests), and the Field Mitigation Guide.
- Manual illustrations are nominal and carry **+/- 1 in** unless noted.
- Fields vary between venues and with wear. "Successful teams will design ROBOTS that are insensitive to these variations."
- **[CODE]** Build tolerance into auto paths and aim (+/- 1 in on everything). Relocalizing with vision is worth it.

### 9.2 FIELD (p62)
- Roughly **144 x 144 in (365.75 x 365.75 cm)**, measured to the inside surfaces of the perimeter walls.
- **36 soft foam TILES**, each about **24 x 24 x 0.59 in** (60.95 x 60.95 x 1.50 cm). Part am-2499 or equivalent.
- Field elements: **1 HIVE Structure** (1 Frame + 2 HIVES, 1 per alliance) and **4 FLOWERS**.
- Full field kit: AndyMark am-5850_Full or a licensed equivalent. Perimeter: am-0481, which is what the manual's illustrations show. Similar perimeters may be used.
- Some events, including Championship, put the field on risers while the ALLIANCE AREAS stay at floor level.
- The perimeter may be anchored, in which case the perimeter straps are not needed.
- All fields at one event must be consistent (T404).

### 9.3 Areas, Zones, & Markings (p63-64)
- A "zone" is inside the FIELD. An "area" is outside the FIELD.
- Tape: **1 in or 2 in** wide ProGaff gaffer tape (or comparable) in red and electric blue. Lines are continuous, and **no tape line crosses a TILE seam** this season.
  - **[CODE]** Line sensors can't rely on seams, though seams also don't confuse the tape.
- **ALLIANCE AREA**: about **97 in wide x 54 in deep**, infinitely tall, outside the field. Includes the tape. The 97 in width is set by tile seams.
  - In my frame: red is **x = -54 to 0, y ≈ 23.5 to 120.5** (Fig 9-2/9-6 show it spanning from about seam 1 to seam 5, centered at y ≈ 72). Blue is the mirror: x = 144 to 198, same Y span.
  - The 5 alliance NECTAR sit in a holder on the wall at about y ≈ 72 (Fig 9-6, Fig 10-2).
- **LOADING ZONE** (Fig 9-3): about **23 in wide (width set by TILE seams) x 11 in deep**, infinitely tall, bounded by tape and the perimeter walls. **Includes the tape lines.** Alliance-specific: it belongs to the alliance with the adjacent ALLIANCE AREA.
  - **RED LOADING ZONE**: against the **red wall (x=0)**, in **TILE A5**: **x ≈ 0-11, y ≈ 96.5-119.5** (between seams 4 and 5). This is the **rear half** of the red side.
  - **BLUE LOADING ZONE**: against the **blue wall (x=144)**, in **TILE F2**: **x ≈ 133-144, y ≈ 24.5-47.5**. This is the **audience half** of the blue side.
  - **[CODE]** These are the PARK targets.
  - The two zones are rotationally symmetric (180° about field center (72,72)), not mirror-symmetric.
- **GARDEN** (Fig 9-3): about **23 in long x 2 in wide**. It is **2 pieces of 1 in tape** laid against the wall, and the zone is defined by the tape's outside edge. It sits in opposite corners.
  - **RED GARDEN**: along the **audience wall** in **TILE A1**: **x ≈ 0-23, y ≈ 0-2**.
  - **BLUE GARDEN**: along the **rear wall** in **TILE F6**: **x ≈ 121-144, y ≈ 142-144**.
  - **[CODE]** Each GARDEN starts with 4 POLLEN (10.3.1). They are placed in a line from the corner nearest the alliance area, touching the audience wall (red) or rear wall (blue).
    - Red estimate: POLLEN centers at about (1.4, 1.4), (4.2, 1.4), (7.0, 1.4), (9.8, 1.4).
    - Blue estimate: (142.6, 142.6), (139.8, 142.6), (137.0, 142.6), (134.2, 142.6).
    - **[DERIVED]** from the 2.8 in ball diameter.

### 9.4 TILE Coordinates (p65)
- **Fig 9-4** labels seam lines: V, W, X, Y, Z (vertical seams, left to right) and 1-5 (horizontal seams, audience to rear).
- **Fig 9-5** labels tiles: columns **A-F** (left/red to right/blue) and rows **1-6** (audience to rear).
- Feature-to-tile map from the figures (in my frame):
  - Red LOADING ZONE: **A5**. Blue LOADING ZONE: **F2**.
  - Red GARDEN: **A1** (audience wall). Blue GARDEN: **F6** (rear wall).
  - FLOWERS sit on seam lines (exact seam center): on the **red wall at seam 2 (y=48)**, the **rear wall at seam W (x=48)**, the **blue wall at seam 4 (y=96)**, and the **audience wall at seam Y (x=96)**.
  - The HIVE structure covers the center of tiles C3/C4/D3/D4 (spanning about x 47-97, y 52-92).

### 9.5 ALLIANCE AREA (p66)
- **Red ALLIANCE AREA is on the left from the primary audience view.**
- Fig 9-6: the area is 97 in long (width set by tile seams) x 54 in deep. A NECTAR holder (5 red NECTAR) sits on the outside of the wall near the field midline. The red LOADING ZONE is at the far end of the area and the red GARDEN is at the audience corner.
- The event may provide tables or stools for OPERATOR CONSOLES. Do not move them without permission from the Head REF, Field Supervisor, or FTA.

### 9.6 HIVE Structure (p67-70)
- **Located in the center of the FIELD.** One frame holds a **red HIVE and a blue HIVE**. Each HIVE has **2 CELLS**, one at each end.
- Each HIVE pivots and is **bi-stable**: one CELL faces up at any time. It holds position until enough POLLEN/NECTAR is LAUNCHED into the **upward-facing** CELL.
- **Each CELL's bottom face carries a unique AprilTag Cluster of 4 tags.**

#### 9.6.1 Frame (Fig 9-8)
- Two triangular metal structures attach to **mounting strips under the TILES**. A crossbar connects them at the apex. The yellow BIOBUZZ logo panel on the frame may not be present at every event.
- **Frame width 49.46 in (125.65 cm)** runs along the crossbar, in X. **Depth 38.95 in (98.95 cm)** is the triangle base, in Y, and is the widest point.
- **Pivot axis height: 43.95 in (111.65 cm) above TILES.** There are 2 pivots, one per HIVE. The pivot axes run parallel to X, so **HIVES tip in the Y-Z plane, toward audience or rear**.
- **[DERIVED]** Frame footprint centered at (72,72): triangles (legs) in the planes **x ≈ 47.3 and x ≈ 96.7**, base extent **y ≈ 52.5 to 91.5**.
  - **[CODE]** These are the leg obstacles for path planning: 4 leg feet near (47.3, 52.5), (47.3, 91.5), (96.7, 52.5), (96.7, 91.5).
  - Fig 9-8 seems to show base rails along Y at each triangle. The cross members along X are labeled "Under TILE Strip". **[AMBIGUOUS]** Whether the Y-direction base rails sit on top of the tiles (a bump or obstacle) is not stated. Check the CAD.
- **[STRATEGY][AMBIGUOUS]** Lowest point of the HIVE is 30.6 in, and the maximum robot height is 29 in (robot rules, other range). The space between the triangles, 49.46 in wide, may be drivable under the HIVE. Check the game rules (G417 HIVE interaction) and actual clearance before planning paths under it.

#### 9.6.2 HIVE and CELL (Fig 9-9, 9-10, 9-11)
- Each HIVE has 2 CELLS of its color plus a connecting assembly that rotates on the pivot at the top of the frame. **Two stable positions.**
- **CELLS are "approximately 18.8 in (47.8 cm) apart"**. Fig 9-9 shows this as **18.84 in (47.85 cm)**, the **gap between the inner faces of the two CELLS**, not center-to-center.
- **Fig 9-9 dimensions** (HIVE shown level):
  - **Overall HIVE length: 42.91 in (109.0 cm)**, outer face to outer face.
  - **Each CELL is 12.04 in (30.58 cm) deep** along the HIVE arm.
  - **Gap 18.84 in** (42.91 = 12.04 + 18.84 + 12.04 ✓).
  - Labeled parts: Pivot (center) and Damper (beside the pivot; it contacts the frame in each stable state).
- **[DERIVED] CELL center-to-center along the arm: 30.88 in**, so each CELL center is **15.44 in from the pivot along the arm**.
- **Fig 9-10 key heights** (stable/tipped state):
  - **The HIVE arm is tilted 30° from horizontal.**
  - **Top of (up) CELL opening: 65.6 in (166.60 cm)** above TILES.
  - **Bottom of (up) CELL opening: 53.5 in (135.90 cm)** above TILES.
  - **Bottom of HIVE (lowest point, on the down CELL): 30.6 in (77.70 cm)** above TILES.
  - **HIVE center to center (red HIVE to blue HIVE, in X): 25.5 in (64.75 cm).**
- **CELL opening (Fig 9-11)**: a pentagon ("house" shape), **20 in (50.8 cm) wide x 14 in (35.55 cm) tall**. The straight side walls are **7.61 in (19.35 cm)** tall, then a peak. Depth is **12 in (30.5 cm)**. A square mounting hole sits at the center of the bottom edge, where the arm tube attaches.
- **Orientation of the opening (from the Fig 9-10 side view and 9-9)**:
  - The opening plane is **perpendicular to the HIVE arm**, at the **outer end** of each CELL. When tipped, the **up CELL's opening faces outward, away from field center in Y, and 30° upward**.
  - The opening plane is 60° from horizontal. Check: 14 in x sin 60° = 12.1 in = 65.6 - 53.5 ✓.
  - So the **red audience-side CELL, when up, opens toward the audience half**, and the far-side CELL, when up, opens toward the rear half.
  - **[CODE][STRATEGY]** Launch at the up CELL from the half of the field on the same side as that CELL.
  - **[AMBIGUOUS]** I inferred this orientation from the drawings, not from text. Verify against the CAD.
- **[DERIVED][CODE] HIVE / CELL positions in my frame** (nominal, +/-1 in plus derivation error ~1-2 in):
  - **Red HIVE pivot**: x = 72 - 12.75 = **59.25**, y = **72**, z = **43.95**. Red CELLS span **x ≈ 49.25-69.25** (20 in wide).
  - **Blue HIVE pivot**: x = 72 + 12.75 = **84.75**, y = **72**, z = **43.95**. Blue CELLS span **x ≈ 74.75-94.75**.
  - The top-view figures agree: red HIVE ≈ x 47.5-70, blue ≈ x 74.5-96.
  - **Up-CELL opening**: the opening's lower edge is about 19.3 in from y=72 and the upper edge about 12.3 in. The pentagon centroid is about **16.5 in from y=72, at z ≈ 58.3**. The CELL interior (box center) is about **11.3 in from y=72, z ≈ 55**.
    - So the up-CELL aim points are roughly **(59.25, 55.5-60.7, ~56-58)** for the red audience CELL and **(59.25, 83.3-88.5, ~56-58)** for the red far CELL. Same for blue with x = 84.75.
  - **Existing code check**: `LaunchTargets.CELL_Y_OFFSET = 9.4` (half of 18.8) is **too small**. The offset of the up CELL from the HIVE center is about **11-16.5 in** depending on aim point (interior vs. opening). The 18.8 in figure is the gap *between* CELLS. `RED_HIVE_X = 60` and `BLUE_HIVE_X = 84` are close to the derived 59.25 / 84.75.
- **Which CELL is up at match start** (Fig 9-17 and Fig 10-2): **red audience-side CELL UP** (contains the 3 red NECTAR) and red far CELL down. **Blue far-side CELL UP** (contains the 3 blue NECTAR) and blue audience CELL down.
  - Memory rule from the manual: "the CELL which 'points at' a FLOWER should be the one tilted down." The red far CELL lines up with the rear-wall FLOWER at x=48. The blue audience CELL lines up with the audience-wall FLOWER at x=96.
  - **[CODE]** Auto must target **red: audience CELL first; blue: far CELL first**. Each TIP swaps which CELL is up, so the target alternates audience/far after every TIP.
- **[STRATEGY]** When a HIVE tips, the formerly-up CELL ends up facing down and outward at 30° below horizontal. Its contents (including the 3 pre-staged NECTAR) will likely **spill onto the field** under that end of the HIVE. G409 (other range) forbids catching elements falling from a tipped HIVE.

### 9.7 FLOWER (p70-71, Fig 9-12)
- **4 FLOWERS attached to the perimeter wall**:
  - **(0, 48)** on the red wall,
  - **(48, 144)** on the rear wall,
  - **(144, 96)** on the blue wall,
  - **(96, 0)** on the audience wall.
  - Positions are at the wall surface and seam line. The opening center sits inward from the wall; see below.
  - These are shared, not alliance-owned. Note the 180° rotational symmetry.
- **Top opening: 4.0 in (10.15 cm) diameter, about 21.5 in (54.6 cm) above TILES.**
- **Backstop** on top to guide elements in: **1.25 in (3.15 cm) tall**, purple, on the wall side.
- **Retrieval Opening** at the bottom for removing elements: **about 3.55 in (9.0 cm) tall x 3.57 in (9.1 cm) deep**.
- **Lower ring**: sits on the TILE floor, **about 0.4 in (1.0 cm)** tall (Fig 9-12 says bottom ring thickness **0.43 in / 1.1 cm**). It has a hole for POLLEN to sit in, **2.79 in (7.1 cm) diameter**.
- Upper and middle rings are connected by **4 HIPS pipes** (green). Middle and lower rings are connected by a square extrusion on the wall side.
- Fig 9-12 plan view of the top ring: center offsets **2.40 in (6.10 cm)** to one side of the housing and **1.89 in (4.80 cm)** to the backstop/wall side, plus a diagonal **1.94 in (4.95 cm)** dimension.
  - **[DERIVED][AMBIGUOUS]** The top opening center is about **1.9-2.5 in out from the wall**. Exact wall-to-center distance: use the CAD.
  - **[CODE]** Placement/aim for NECTAR in endgame: FLOWER opening ≈ (≈2.2, 48, 21.5), (48, ≈141.8, 21.5), (≈141.8, 96, 21.5), (96, ≈2.2, 21.5).
- **[CODE]** POLLEN can be pulled out of the bottom Retrieval Opening, which is a POLLEN source. Each FLOWER starts with 4 POLLEN, and the bottom one rests in the 2.79 in hole.
  - NECTAR (3.6 in) does **not** fit the 2.79 in ring hole. Whether it fits through the 3.55 in-tall retrieval opening is borderline. **[AMBIGUOUS]**

### 9.8 SCORING ELEMENTS (p72)
- **POLLEN**: about **2.8 in (7.1 cm)** Gopher ResisDent polyethylene ball, **yellow**, am-5851_yellow (wiffle-style holes).
- **NECTAR**: about **3.6 in (9.1 cm)** Gopher ResisDent ball, **red** (am-5852_red) and **blue** (am-5852_blue).
- **Totals per MATCH: 40 POLLEN, 8 red NECTAR, 8 blue NECTAR.**
- Not perfectly spherical, and sizes vary. **[CODE]** Intake and launch tuning must tolerate size variation. Use color sensing to tell POLLEN (yellow) from NECTAR and alliance colors apart. Two ball sizes affect launch RPM per element type.

### 9.9 AprilTags (p72-75)
- **Tags are 3.25 in (8.25 cm) square, family 36h11**, used for navigation and targeting.
- **Clusters**: 4 tags on one sticker, each with an "ID" text label, aligned to the CELL using **Reference Holes**, which can be used to measure the cluster location relative to the field.
- **Fig 9-15 cluster layout** (in the cluster's own frame, reading the tags upright):
  - Tags in a row of 4. **Tag centers at -6.5, -2.75, +2.75, +6.5 in** from the cluster centerline (6.5 in / 16.50 cm and 2.75 in / 7.0 cm).
  - Tag spacing: 3.75 in between outer and inner tag, 5.5 in between the two inner tags. There is a 2.25 in gap in the middle where the HIVE arm tube runs (Fig 9-16).
  - The tag-row horizontal centerline is **2.75 in (7.0 cm)** above the Reference Hole line.
  - **Reference holes: 0.5 in (1.25 cm) diameter**, at **±7.0 in (17.80 cm)** from the cluster centerline (14 in apart), on the bottom edge of the sticker.
  - **Centerline of Reference Holes is 9.938 in (25.25 cm) from the "Front of Cell"** (the opening rim). So the **tag centers are about 7.19 in from the front (opening) edge** of the CELL along the bottom face **[DERIVED]**.
  - A colored strip on the sticker's bottom edge reads e.g. "RED SCORING Tag family: 36h11" (cluster 30-33) or "RED AUDIENCE Tag family: 36h11" (cluster 34-37, Fig 9-16).
  - **ID order left to right when reading the cluster upright: 30, 31, 32, 33** (confirmed in the high-res Fig 9-15).
- **Mounting**: each cluster sits on the **bottom of a CELL, facing downward toward the TILES**, with its **bottom edge (the reference-hole edge) toward field center** (Fig 9-16).
  - Because the cluster is on the CELL bottom face and that face is parallel to the tilted arm, **the tag plane is tilted 30° from horizontal**. **[DERIVED]**
  - **Up CELL**: tags face **down and 30° outward** (away from field center in Y).
  - **Down CELL**: tags face **down and 30° toward field center**.
- **Tag IDs and locations** (Fig 9-17 labels: "Red Scoring Tags", "Red Audience Tags", "Blue Scoring Tags", "Blue Audience Tags"):

  | Cluster | IDs | CELL | Fig 9-17 order, field-left (-X) to right (+X) | Up at start? |
  |---|---|---|---|---|
  | "Red Scoring" | **30, 31, 32, 33** | red CELL, **rear side** (far from audience) | **33, 32, 31, 30** | **No (down)** |
  | "Red Audience" | **34, 35, 36, 37** | red CELL, **audience side** | **34, 35, 36, 37** | **Yes (up)** |
  | "Blue Audience" | **38, 39, 40, 41** | blue CELL, **audience side** | **38, 39, 40, 41** | **No (down)** |
  | "Blue Scoring" | **42, 43, 44, 45** | blue CELL, **rear side** | **45, 44, 43, 42** | **Yes (up)** |

  - The reversed order on the rear clusters is consistent with every cluster reading ascending in its own frame, with the bottom edge toward center.
- **[DERIVED][CODE] Approximate tag-center coordinates (nominal start state)**:
  - The **X of each tag** = HIVE X + offset (-6.5, -2.75, +2.75, +6.5), in the field-left-to-right order above.
  - Red rear cluster: **ID33 x≈52.75, ID32 x≈56.5, ID31 x≈62.0, ID30 x≈65.75**.
  - Red audience: **ID34 52.75, ID35 56.5, ID36 62.0, ID37 65.75**.
  - Blue audience: **ID38 78.25, ID39 82.0, ID40 87.5, ID41 91.25**.
  - Blue rear: **ID45 78.25, ID44 82.0, ID43 87.5, ID42 91.25**.
  - **Y and Z depend on whether the CELL is up or down.** Estimates use the arm at 30°, tag centers 14.27 in from the pivot along the arm, and the bottom face about 1.4 in below the pivot (back-solved from the 53.5/65.6 heights):
    - **Up CELL tags**: about **13.1 in from y=72**, so **y ≈ 58.9 (audience) / 85.1 (rear)**, **z ≈ 49.9 in**. Normal is down, tilted 30° away from center.
    - **Down CELL tags**: about **11.7 in from y=72**, so **y ≈ 60.3 (audience) / 83.7 (rear)**, **z ≈ 35.6 in**. Normal is down, tilted 30° toward center.
  - **[CODE]** The Limelight must look **up** at about 30-50 in tag height. Tag pose depends on HIVE state, which **changes with every TIP**, so a fixed-field-map localization will be wrong for the tags on a HIVE that has moved.
    - Options: (a) keep two field maps per HIVE (up/down) and track the TIP count, (b) infer state from which tags are visible and their apparent height or tilt, (c) use tag-relative targeting of the up CELL rather than global relocalization.
    - Also the tags rotate 60° between states (+30° vs -30°).
  - **[CODE] Targeting**: the up CELL's own tags are on its underside, near the arm and toward the pivot. A robot in the up CELL's half of the field, looking toward center and up, sees them tilted toward it (they face outward), which is good for aim.
  - **[AMBIGUOUS]** The manual says "located in the following locations ... in the order indicated in Fig 9-17". Exact 3D coordinates are not published, so use the CAD and Reference Holes to get them precisely. My Y/Z values are derived and probably within ±1.5 in.
  - Images in the manual are not to scale. Print tags from the Playing Field Resources page.

### 9.10 FIELD STAFF (p76)
- **Head REFEREE**: final authority on scores, fouls, and cards.
- **FTA**: technical issues, team advocate.
- **FIELD Supervisor**: match flow and field reset. May be the same person as the FTA or Head REF at small events.

### 9.11 Event Management System (p77), Table 9-1 Audio cues
- Audio cues are a courtesy. **The visual field timer is the authority.**

| Event | Timer | Cue |
|---|---|---|
| MATCH start | 2:30 | "This MATCH begins in 3, 2, 1, GO" (optional), "Cavalry Charge" |
| AUTO ends | 2:00 | "Buzzer x 3" |
| AUTO to TELEOP transition | 0:08 to 0:01 | "Drivers, pick up your controllers, 3-2-1" |
| TELEOP begins | 2:00 | "3 Bells" |
| **FLOWER Ownership Unlocked** | **1:00** | "Chime" |
| Final 20 s | 0:20 | "Train Whistle" |
| MATCH end | 0:00 | "3-second Buzzer" |
| MATCH stopped | N/A | "Foghorn" |

- **[CODE]** TeleOp should show a match-time HUD and rumble at the following times, measured from TELEOP start:
  - **60 s (FLOWERS / NECTAR unlock)**: TELEOP 1:00 remaining = 60 s into TELEOP.
  - **100 s (final 20 s)**: TELEOP 0:20 remaining.
  - **115-120 s**: wrap up and return to the LOADING ZONE.

---

## Section 10 Game Details (p79-95)

### 10 intro / 10.1 MATCH Overview (p79)
- Fig 10-1: isometric field from the red-alliance side.
- MATCH sequence: pre-MATCH setup, **30 s AUTO**, **8 s transition**, **2:00 TELEOP**, post-MATCH reset.
- Robots collect POLLEN/NECTAR to (a) move it to their **GARDEN**, (b) place it into **FLOWERS**, and (c) **LAUNCH** it into their CELLS to cause a **HIVE TIP**.
- **Each time a HIVE is TIPPED, the alliance may enter one of the 5 NECTAR** staged in its ALLIANCE AREA. **With 60 s left, the alliance may enter all remaining NECTAR.**
  - **[STRATEGY]** TIPS unlock NECTAR, and NECTAR is the key to FLOWER ownership.
- The MATCH ends with robots claiming FLOWER ownership with NECTAR and **returning to their LOADING ZONE**.

### 10.2 DRIVE TEAM (p79-80)
- Up to **4 people** from the same team, with only **1 non-STUDENT** allowed.
- The intent is to prevent "adopting" members of other teams for strategic advantage. Exceptions are allowed in extenuating circumstances, such as a delayed bus.
- **Table 10-1 roles**:
  - **DRIVE COACH**: max 1. Any team member, may be an adult. Wears a "DRIVE COACH" badge.
  - **DRIVER** (operates the robot) and **HUMAN PLAYER** (SCORING ELEMENT manager): max 3 combined. Must be STUDENTS wearing "DRIVE TEAM" badges.
  - The extracted table layout is merged; the max-3 most likely covers DRIVERS plus HUMAN PLAYERS. **[AMBIGUOUS]**
- A STUDENT is someone who has not finished high school or equivalent as of **September 1** of the season.

### 10.3 Setup (p80-83)

#### 10.3.1 SCORING ELEMENTS (Fig 10-2, p81-82)
- **Each HIVE starts tilted with one CELL down and one up.** "The CELL which 'points at' a FLOWER should be the one tilted down." That means the red far CELL and the blue audience CELL are down, and **red audience and blue far CELLS are up** (see 9.6.2).
- **40 POLLEN**:
  - **4 in each of the 4 FLOWERS (16)**,
  - **4 in the red GARDEN**,
  - **4 in the blue GARDEN**,
  - **4 pre-loaded in each ROBOT (16)**.
  - Preloads start in or on the robot, or on the TILES touching the robot.
  - If a robot is absent, its preloads go at about the **center of the LOADING ZONE against the wall**.
  - GARDEN POLLEN sits in a line from the corner closest to the ALLIANCE AREA, touching the audience wall (red) or rear wall (blue).
- **NECTAR (8 per color)**:
  - **3 in each upward-facing CELL of its color (6 total)**. They are placed against the CELL's back wall, in a line along the side closest to that color's ALLIANCE AREA.
  - **5 in each ALLIANCE AREA (10 total).**
- Fig 10-2 labels:
  - "ROBOT Pre-loads -RED [8] POLLEN" is drawn along the red wall at the red LOADING ZONE.
  - "ALLIANCE AREA-RED [5] NECTAR" sits on the red wall at y≈72.
  - "FLOWER [4] POLLEN" appears at each of the 4 FLOWERS.
  - "GARDEN-RED [4] POLLEN" sits at the audience/red corner. "GARDEN-BLUE [4] POLLEN" sits at the rear/blue corner.
  - "CELL Upward Tilted [3] NECTAR" labels the red audience CELL and the blue rear CELL.
- **[AMBIGUOUS]** Fig 10-2 has a typo: the label pointing at the **blue** far CELL reads "CELL -RED Upward Tilted [3] NECTAR -BLUE". It should say CELL-BLUE.
- Fig 10-2 draws the robot preloads along the wall at the LOADING ZONE. That is an illustration only. Robots start per G304 (other range).
- Championship and Premier events may change the number, type, and distribution of elements (15.2). Championship changes are published by the last scheduled Team Update.

#### 10.3.2 DRIVE TEAMS (p82)
- Drive teams stage after the previous drive team leaves. Delaying risks a **G301** violation.
- Only assigned and inspected members may stage, in their ALLIANCE AREA, with badges above the waist.
- If teams disagree on positions, **"Red 1"/"Blue 1" stands closest to the audience**.
- In Playoffs, the ALLIANCE CAPTAIN wears an identifier.

#### 10.3.3 OPERATOR CONSOLES (p82)
- Must comply with 12.9.
- **[CODE]** To run AUTO, select the OpMode with the **30-second timer enabled**. Otherwise select a TELEOP OpMode. In both cases **press INIT**.
  - Auto OpModes should use `@Autonomous(preselectTeleOp = ...)` so TeleOp is queued for the transition.

#### 10.3.4 ROBOTS (p82-83)
- Stage per **G304**. Delaying risks G301.
- **Robots must start the MATCH contacting 4 pre-loaded POLLEN**, held in or on the robot, or on the TILES **fully on its own ALLIANCE side** and touching the robot.
  - **[CODE]** Auto assumes 4 POLLEN loaded. Intake logic should start with count = 4.
- A DISABLED robot may be removed before the match with FIELD STAFF. A team whose robot is disabled or absent still gets match points if the robot passed inspection and at least 1 STUDENT drive team member is present.
- **Placement order** (if either alliance requests it, via the Head REF): first red, then first blue, then second red, then second blue.
  - In Qualification matches, Red 1 / Blue 1 places first. In Playoffs, the alliance lead decides.
  - **[STRATEGY]** Placing last lets you react to the opponent's start position. Have multiple auto start positions ready.

### 10.4 MATCH Periods (p83)
- **AUTO: 30 s (0:30)**, no driver input.
- **8 s transition** between AUTO and TELEOP, for scoring purposes (10.5).
- **TELEOP: 2:00.** See Table 9-1.

### 10.5 Scoring (p84-89)
- Rewards come as MATCH points and RANKING POINTS (RP), with ranking per 13.6.3. All achievements update throughout the match.
- **Assessment timing**:
  - A. **HIVE TIPS** are assessed throughout the match, until all elements and robots come to rest after the match ends.
  - B. **TIPS completed before TELEOP starts count as AUTO.** Because TELEOP starts after the 8 s transition, a TIP that completes during the transition probably counts as AUTO. **[AMBIGUOUS]** Achievements during the transition may be penalized (see below).
  - C. **POLLEN/NECTAR remaining in a CELL** are assessed after everything comes to rest at match end.
  - D. **FLOWER** scoring is assessed throughout, with the final assessment at end of TELEOP after everything comes to rest.
  - E. **GARDEN** is assessed at end of TELEOP, at rest.
  - F. **LEAVE and AUTO PARK** are assessed at the **end of AUTO**.
  - G. **TELEOP PARK** is assessed at the **end of the MATCH**.
- Scoring is done by human volunteers. Live-score errors are not ARENA FAULTS. Make achievements obvious and unambiguous.
- **Achievements scored before the match starts, during the AUTO-to-TELEOP transition, or after 0:00 may be penalized.**
  - **[CODE]** Auto must stop launching and moving before the 30 s mark. Don't let a queued launch fire late.

#### 10.5.1 HIVE Scoring Criteria (p85)
- **HIVE TIP** happens when (A) the HIVE moves from one stable state to the other, with the down CELL becoming the up CELL, **and** (B) the damper that was previously not touching the frame then **contacts the frame** (Fig 10-3 shows no-contact vs. contact).
  - Volunteers judge "obvious" state changes and don't watch the damper instant.
- **Launching at the down-facing CELL while a HIVE is tipping may disrupt the TIP**, so it might not count.
  - **[CODE]** After the expected TIP, pause launching briefly and re-target the newly up CELL. A target switch is needed after every TIP, so track TIP count or HIVE state, possibly by vision.
- **Launching into the upward-facing CELL is the ONLY allowed way to earn a TIP.** No pushing or touching (G417).
- **POLLEN and NECTAR remaining in CELL**: elements left in an upward-facing CELL at match end score for that alliance.
- **[AMBIGUOUS]** The number of elements required to tip is not specified in this range. "Enough" is all it says, and it depends on the physical HIVE balance and probably the mass in the CELL. A 3.6 in NECTAR is heavier than POLLEN. The 3 NECTAR pre-staged in the up CELL contribute to the start state.

#### 10.5.2 FLOWER Scoring Criteria (p85-87)
- NECTAR and POLLEN score when they are **at least partially within the FLOWER scoring volume**: the space **between the top ring and the middle ring** (purple in CAD Reference 10-4).
  - Elements that dropped below the middle ring (into the lower section or the bottom ring) do **not** count. Elements resting **above** the top ring but partly inside the volume do count (Fig 10-5 H).
- **FLOWER scoring can't begin until 1 minute remains (G410).** Earlier achievements are still scored but penalized.
- **Placing elements into the top is the only allowed way to score.** Follow G418 when interacting with a FLOWER.
- **Bottom NECTAR Bonus**: the alliance whose NECTAR is the **bottom-most** scoring NECTAR in the FLOWER gets points (5).
- **FLOWER Owner**: the alliance whose NECTAR is the **top-most** scoring NECTAR **owns** the FLOWER. It gets **2 points for every POLLEN and NECTAR in the scoring volume, no matter who placed them**.
- **Fig 10-5 examples** (a check means inside the volume, an X means outside):
  - A: blue top, blue bottom, POLLEN between. Blue gets both the owner and the bottom bonus.
  - B: a single blue NECTAR low in the volume is both TOP and BOTTOM.
  - C: red top, blue bottom. A POLLEN below the middle ring is not counted.
  - D: a red NECTAR straddling the top ring is the only NECTAR, so it is both TOP and BOTTOM. Lower POLLEN is outside.
  - E: blue top, red bottom.
  - F: red top, blue bottom. A red NECTAR below the middle ring is not counted.
  - G: a red NECTAR fully above the top ring is outside the volume. The top is the next one down. The bottom is red. Elements below the middle ring are not counted.
  - H: a blue NECTAR resting on the top ring with a small part inside (magnified view) counts as TOP.
- **[STRATEGY]**
  - The **first NECTAR into a FLOWER** gets the bottom bonus (5), and it stays bottom unless pulled out.
  - The **last NECTAR on top** gets ownership: 2 x (count of all elements in the volume). That count includes the 4 pre-staged POLLEN if they stay in the volume.
    - Note: FLOWERS start with 4 POLLEN. Some may sit in the lower ring or retrieval area, outside the volume.
  - POLLEN can be pulled from the bottom Retrieval Opening, but adding elements must happen from the top.
  - The 4 in. top opening takes a 3.6 in NECTAR with only 0.4 in clearance. **[CODE]** NECTAR placement needs a precise end-effector and alignment. The backstop helps.

#### 10.5.3 GARDEN Scoring Criteria (p87-88)
- Elements must be **at least partially in the GARDEN zone**, measured from the outside edge of the 2 in tape. Fig 10-6 shows that balls not over the tape don't count.
- **GARDENS are alliance-specific. They score for the color of the GARDEN, regardless of who placed the elements.**
- **They are not protected.** Either alliance can remove elements from either GARDEN.
- Both alliances' NECTAR and all POLLEN score for the GARDEN's color.
- Fig 10-6 examples: balls on top of or overlapping the blue tape count. Balls off the tape, or beyond the tape's end at the corner, don't.
  - One yellow ball in the corner past the tape end is marked X. Another yellow at the right (wall) end that overlaps the tape is marked as counting.
- **[STRATEGY]** The 4 POLLEN pre-staged in your GARDEN are worth 1 point each at end only if they are still there. They are also a nearby POLLEN source; using them trades 1 GARDEN point for possibly 2+ points elsewhere.

#### 10.5.4 ROBOT Scoring Criteria (p88)
- **LEAVE**: the robot moves so it is **no longer contacting the perimeter wall**, assessed at end of AUTO.
  - **[CODE]** Auto start pose is against the wall. End auto off the wall.
  - **[AMBIGUOUS]** A robot parked in the LOADING ZONE while touching the wall at end of AUTO probably loses LEAVE. Park in the zone without touching the wall.
- **PARK**: the robot is **at least partially in the LOADING ZONE**, and the zone includes its tape lines (Fig 10-7).
  - Examples that count: a robot just overlapping the tape, and a robot fully inside.
  - Example that doesn't count: a robot near the zone but not touching it or its tape.
  - AUTO PARK is assessed at end of AUTO. TELEOP PARK is assessed at end of MATCH.
- **[CODE]** Path end-targets:
  - Red LOADING ZONE center ≈ (5.5, 108). Being partially in is enough, e.g. a robot center at x ≈ 15-18 with an edge over the tape at x ≤ 11.
  - Blue LOADING ZONE center ≈ (138.5, 36).

#### 10.5.5 Point Values (Table 10-2, p89)

| Item | AUTO | TELEOP | RP |
|---|---|---|---|
| LEAVE | **3** | - | - |
| PARK (in LOADING ZONE) | **5** | **5** | - |
| HIVE TIP | **20** | **20** | - |
| POLLEN and/or NECTAR remaining in (up) CELL | - | **2** each | - |
| FLOWER: Bottom NECTAR Bonus | - | **5** | - |
| FLOWER: POLLEN and/or NECTAR in an owned FLOWER | - | **2** each | - |
| GARDEN: POLLEN and/or NECTAR in GARDEN | - | **1** each | - |
| **SWARM RP**: combined LEAVE + PARK points at or above threshold | | | **1** |
| **POLLINATOR 1 RP**: number of TIPS at or above threshold | | | **1** |
| **POLLINATOR 2 RP**: number of TIPS at or above threshold | | | **1** |
| WIN | | | **3** |
| TIE | | | **1** |

- "Remaining in CELL" and FLOWER/GARDEN are assessed at end and listed only under TELEOP.
- TIPS are worth the same in AUTO and TELEOP, so there is no AUTO bonus.

**Table 10-3 RP thresholds**

| RP | FIRST Championship | Regional Championships | All Other Events* |
|---|---|---|---|
| SWARM | TBA | TBA | **16 points** |
| POLLINATOR 1 | TBA | TBA | **4 TIPS** |
| POLLINATOR 2 | TBA | TBA | **7 TIPS** |

- Regional Championship and Championship thresholds will come in Team Updates. *Premier events set their own.
- **[STRATEGY]**
  - Maximum RP per match = 3 (win) + 1 (SWARM) + 1 + 1 (POLLINATOR 1 & 2) = **6**.
  - **SWARM 16** of a maximum 26 (2 x LEAVE 3 + 2 x AUTO PARK 5 + 2 x TELEOP PARK 5). Ways to reach it:
    - both robots LEAVE and AUTO PARK = 16;
    - both LEAVE + both TELEOP PARK = 16;
    - both AUTO PARK + 1 TELEOP PARK = 15, which is not enough without a LEAVE.
    - **Reliable LEAVE + AUTO PARK for both robots alone gets SWARM.**
  - **POLLINATOR**: 7 TIPS per alliance per match for both RPs. That is about 140 points of TIPS. TIPS are counted per alliance's HIVE. **[AMBIGUOUS]** I assume TIPS of your own HIVE only. Only your alliance can launch into your CELLS per other rules; check G-rules.
  - Each TIP also unlocks a NECTAR (max 5) for endgame FLOWERS.
  - TIPS (20 points) dominate scoring, so launch throughput and fast re-targeting after each TIP is the main software priority.

### 10.6 Violations (p90-94)
- Durations:
  - **MOMENTARY**: under about 3 s.
  - **CONTINUOUS**: over about 10 s.
  - **REPEATED**: more than once per match.
  - Referees don't count aloud.
- **STRATEGIC**: aimed at competitive advantage, including deliberate or reckless acts with foreseeable consequences. Accidents aren't strategic, but **exploiting an accident is**. Multiple warnings in one match or across matches for the same violation draw scrutiny.
- Penalties are per instance, one action can violate multiple rules, and calls are made as the referee perceives them.
- **Table 10-4**:
  - **VERBAL WARNING**.
  - **MINOR FOUL = 5 points to the opponent.**
  - **MAJOR FOUL = 20 points to the opponent.**
  - **YELLOW CARD**: a warning. A second YELLOW in the same tournament phase becomes a RED.
  - **RED CARD**: the team is DISQUALIFIED for the match.
  - **DISABLED**: the robot is stopped for the rest of the match.
  - **DISQUALIFIED**: 0 match points and 0 RP in Qualification. In Playoffs the alliance gets 0.
- **10.6.1 Cards**:
  - YELLOW/RED for CIC behavior per the Escalation Guidelines.
  - A team with any card carries a YELLOW into later matches. Cards are rescinded if caused by an ARENA FAULT.
  - Yellows are additive: the second becomes RED, even within one match.
  - Display: yellow rectangle = YELLOW, red over yellow = second YELLOW, red = RED. The team number shows a yellow background in later matches.
  - "Subsequent" means chronologically later, even if matches run out of order.
  - **All YELLOW CARDS and G301 VERBAL WARNINGS clear after Practice, Qualification, and division Playoff matches.** Other Head-REF verbal warnings clear after Practice and persist from Quals onward unless stated otherwise.
- **10.6.2 Card application (Table 10-5)**:
  - Before Quals: the Head REF may carry an egregious warning or YELLOW to the first Qual match.
  - During Quals: applied to the current or just-completed non-SURROGATE match. For a surrogate match, it goes to the previous Qual match.
  - Between Quals and Playoffs: applied to the alliance's first Playoff match.
  - During Playoffs: the current or just-completed match.
  - A match stops being "current" when results are posted or robots are released, whichever is later.
- **10.6.3 Playoffs**: cards apply to the **whole alliance**. 2 YELLOWS = RED, and the alliance is DQ'd for that match.
- **10.6.4 Table 10-6 examples**:
  - "MINOR FOUL per SCORING ELEMENT" means N fouls for N elements in violation.
    - **[CODE]** This is relevant to element-control limits (G407 in another range). Over-controlling by several elements multiplies fouls.
  - "MAJOR per instance + YELLOW if REPEATED": the 2nd instance gives another MAJOR plus a YELLOW. Further instances add MAJORs but no more YELLOWS for that match.
  - "MAJOR + additional MAJOR per 3 s uncorrected": 15 s of violation = **6 MAJORS** (120 points).
  - "VERBAL WARNING; MAJOR + YELLOW if STRATEGIC."
  - "MAJOR per MATCH; MAJOR + YELLOW per MATCH if STRATEGIC": at most 1 MAJOR and 1 YELLOW per team per match.
  - "VERBAL WARNING; YELLOW if subsequent violations during the event": YELLOW for each later instance, in the same or a later phase.
  - "VERBAL WARNING; MAJOR + YELLOW per instance if STRATEGIC; MAJOR + RED per instance if STRATEGIC and either CONTINUOUS (≥10 s entanglement) or the opponent can't drive": 1 MAJOR and 1 card per instance, and multiple instances can add up.

### 10.7 Head REFEREE (p95)
- Has ultimate arena authority (T401) and may take input from FIRST, FTA, ED, and staff. **Rulings are final.**
- **No review of video, photos, or renderings from any source, under any circumstances.**

### 10.8 Other Logistics (p95)
- **10.8.1** The FTA, LRI, or Head REF may bar an unsafe robot from Practice matches.
- **10.8.2 SCORING ELEMENT logistics**:
  - **POLLEN that exits the field** is reintroduced by FIELD STAFF at the earliest safe opportunity, at the nearest convenient location.
  - **NECTAR that exits the field** is returned to **that alliance's DRIVE TEAM** for reintroduction per **11.4.6 Human** (other range).
    - **[STRATEGY]** Losing NECTAR off-field is not permanent, but it costs time.
  - Reasonable delays in reintroducing elements are not an ARENA FAULT.
  - An ARENA FAULT is not called for matches that accidentally start with damaged, wrong-count, or misplaced elements. Damaged elements are replaced only at the next reset. **Drive teams should alert FIELD STAFF before the match starts.**
    - **[STRATEGY]** Check the start-state checklist before each match: CELL orientation, 3 NECTAR in the up CELL, 4 POLLEN per FLOWER and GARDEN.
- **10.8.3** FIELD STAFF may apply Field Mitigation Guide steps during a match.
- **10.8.4 Field reset**: after the Head REF signals, teams retrieve robots and consoles, the next teams load, and staff reset elements.

---

## Summary of the most consequential items for software

1. **Up CELL at start**: red = audience-side CELL (tags 34-37), blue = far-side CELL (tags 42-45). Each TIP swaps the up CELL, so the target and the visible tag set alternate.
2. **CELL geometry**:
   - The 18.84 in figure is the **gap** between CELLS. The CELL center is 15.44 in from the pivot along a 30° arm.
   - The up-CELL opening is at z ≈ 53.5-65.6 in and about 12-19 in from the HIVE center in Y, facing outward and 30° up.
   - Red HIVE x ≈ 59.25, blue ≈ 84.75, pivot z = 43.95.
   - `LaunchTargets.CELL_Y_OFFSET = 9.4` should be about 11-16 in, and the aim height is about 55-58 in.
3. **AprilTags**:
   - 36h11, 3.25 in, on CELL **undersides**, facing down and tilted ±30°. Tag offsets within a cluster: ±2.75 and ±6.5 in.
   - Tags move when the HIVE tips: z ≈ 50 in when the CELL is up, ≈ 35.6 in when down.
   - Static field-map relocalization from these tags must account for HIVE state.
4. **Field features**:
   - LOADING ZONES: red x 0-11, y 96.5-119.5; blue x 133-144, y 24.5-47.5.
   - GARDENS: red x 0-23, y 0-2; blue x 121-144, y 142-144.
   - FLOWERS at (0,48), (48,144), (144,96), (96,0). Top opening 4 in diameter at 21.5 in height.
   - HIVE frame legs at about x 47.3 / 96.7, y 52.5-91.5.
5. **Scoring and RP**:
   - TIP = 20 (both periods). SWARM 16 = both robots LEAVE + AUTO PARK. POLLINATOR at 4 and 7 TIPS.
   - FLOWER/NECTAR only in the last 60 s. Nothing may score during the 8 s transition or after 0:00 without penalty risk.

---

# BIOBUZZ Manual (TU07) -- Notes, Pages 96-130

Coverage: p96 is blank (verified by rendering). p97-115 = **Section 11 Game Rules (G)** (all of G101-G428).
p116-130 = **Section 12 ROBOT Construction Rules (R)**, from the intro through R502 (servos). R503+ (motor count
limit, power regulation R505, etc.) start after p130 and are NOT covered here.

**No scoring tables or ranking-point thresholds appear in pages 96-130.** Scoring (Section 10) ends before p96.
Scoring-relevant references in this range: 10.3.1 SCORING ELEMENTS (pre-loads), 10.5.2 FLOWER Scoring Criteria,
10.6.1 YELLOW and RED CARDS, 10.8 Other Logistics, T402/T406 (tournament rules).

Legend: **[CODE]** = constrains robot software. **[STRATEGY]** = affects match strategy. `*` before a rule title in the
manual marks a "evergreen"/general rule (manual's own asterisk notation, kept as-is).
"STRATEGIC" (defined elsewhere in the manual) = deliberate violation that benefits the team, so escalated penalties apply.

---

## 11 Game Rules (G)

### 11.1 Personal Safety (p97)

**G101 \*Humans, stay off the FIELD during the MATCH.** No team member may enter the FIELD during a MATCH.
- Penalty: VERBAL WARNING.
- Blue box: Stay outside the FIELD perimeter and don't affect the MATCH except as 11.4.6 Human allows. Stepping onto
  the FIELD or hanging a large portion of the body over it violates the rule. Pointing, waving, or leaning over the wall
  without affecting the MATCH does not.

**G102 \*Be careful when interacting with ARENA elements.** Team members may not (A) climb on, (B) hang from,
(C) manipulate an ARENA element so it doesn't return to its original shape without human help, or (D) damage it.
- Penalty: VERBAL WARNING.
- Blue box: DRIVE TEAM may brace the FIELD perimeter at any time. Moving the perimeter out of position violates G102.C.

### 11.2 Conduct (p97-99)

**G201 \*Be a good person.** Be civil and respectful of people and equipment (FIRST Code of Conduct, Core Values, 1.4.2).
- Penalty: VERBAL WARNING; YELLOW CARD for later violations at the event.
- Blue box: Particularly contemptible behavior can mean ARENA ejection (individuals), DQ (teams), and further escalation.

**G202 \*Follow the Competition Integrity Contract (CIC)** (Section 1.5) in every MATCH. Example violations:
(A) deliberately violating the spirit of the rules; (B) throwing MATCHES, deliberately missing RPs, or playing below
ability to manipulate rankings; (C) asking other teams to do B; (D) asking teams not to show up; (E) distracting,
disrupting, or taunting opponents; (F) disrupting ARENA operations.
- Penalty: VERBAL WARNING; YELLOW or RED CARD for later violations at the event.
- Blue box: Alliances may still plan unconventional roles in specific MATCHES if they intend to play the game. Card
  color is at the Head REFEREE's discretion based on severity and impact.
- **[STRATEGY]** You may not deliberately skip RPs to manipulate rankings.

**G203 \*Show up to your MATCHES.** Once the ROBOT passes initial complete inspection, at least 1 DRIVE TEAM member
must report to and take part in each assigned Qualification MATCH.
- Penalty: DISQUALIFIED from the current MATCH.
- Blue box: Tell the Lead Queuer if the ROBOT can't play. Ask FIELD STAFF about accommodations in extenuating cases.

**G204 \*Do not expect to gain by doing others harm.** Actions clearly aimed at forcing the opponent to break a rule are
not allowed. The targeted ALLIANCE is not penalized for violations forced this way.
- Penalty: MAJOR FOUL per instance; MAJOR FOUL per instance + YELLOW CARD per MATCH if REPEATED. No penalty to the
  forced ALLIANCE.
- Blue box: Doesn't apply to strategies consistent with standard gameplay. Requires a deliberate act the victim has
  little or no chance to avoid. Examples:
  - (A) red ROBOT drives into a blue ROBOT lined up to score NECTAR into a FLOWER with 65 s left, causing an early
    NECTAR drop (would break G410);
  - (B) red ROBOT drives through the blue LOADING ZONE while a blue human is introducing NECTAR, deflecting it before
    it touches the TILE (would break G427).
- **[STRATEGY]** Protects you from being shoved into early-NECTAR or loading-zone penalties.

**G205 \*Egregious or exceptional violations.** Egregious behavior beyond the listed rules, or repeated violations of
any rule or procedure, is prohibited. The Head REFEREE may give a YELLOW or RED CARD for egregious ROBOT actions or
team behavior at any time. Continued violations go to FIRST HQ (possible removal from awards or the event).
Deliberately breaking a rule because the benefit outweighs the penalty is egregious (e.g., repeated violations of
rules with no escalation path). Breaking a YELLOW-CARD-only rule in an ALLIANCE's possible last MATCH before cards
clear (10.6.1) or the end of the event (last qual, lower-bracket match) gets extra scrutiny and **will likely be a
RED CARD**.
- Penalty: YELLOW or RED CARD.
- Blue box: Gives the Head REF flexibility. A card may be given for a single instance or multiple instances of one rule.
  Any rule can escalate to a card. The Head REF has final authority.
- **[STRATEGY]** "Take the foul because it's worth it" is itself carded.

### 11.3 Pre-MATCH (p100-103)

**G301 \*Don't delay MATCHES.** DRIVE TEAM may not significantly delay the MATCH start. This only applies once both
(A) the expected start time has passed and (B) the DRIVE TEAM has ARENA access. A significant delay means the Head REF
sees the team as neither MATCH ready (G303) nor making a good-faith effort to get there.
- Expected start: Quals = scheduled time or ~3 min after the previous MATCH on that FIELD, whichever is later.
  Playoffs = scheduled time or 8 min from either ALLIANCE's previous MATCH, whichever is later. Back-to-back MATCHES
  fall under T406 (possibly longer minimum break).
- A team with 1 DRIVE TEAM member present that has told FIELD STAFF the ROBOT won't play counts as MATCH ready.
- Penalty: VERBAL WARNING; MAJOR FOUL for the upcoming MATCH on a later violation in the same tournament phase. In
  Playoffs, the VERBAL WARNING goes to the whole ALLIANCE. If not MATCH ready **within 2 minutes** of the
  warning/foul with no good-faith effort, the ROBOT is DISABLED.
- Blue box: Head REF starts a 2-minute timer and tries to share its status. Good-faith efforts aim only at becoming
  MATCH ready (not significantly altering capability). Examples: (A) walking safely to the FIELD with a ROBOT not being
  modified; (B) quick fixes (tape, cable ties) for STARTING CONFIGURATION compliance; (C) waiting for the DRIVER
  STATION to boot; (D) working with the FTA or field tech staff for a reasonable time; (E) a MOMENTARY "wiggle test" to
  confirm DS-to-RC communication. **During the wiggle test the ROBOT should not drive or touch SCORING ELEMENTS (except
  its pre-loaded POLLEN).**
- **[CODE]** Keep any comms-check routine to a momentary mechanism twitch with no drivetrain motion. Fast boot and
  OpMode load times help avoid delays.

**G302 \*Limit what you bring to the FIELD.** Besides the ROBOT and OPERATOR CONSOLE, items must fit in the ALLIANCE
AREA, be worn or held by the DRIVE TEAM, or be an accommodation (crutches, cushions, kneeling mats, locked non-rolling
single-step stools up to 12 in / 30.5 cm). No item may: (A) disrupt ARENA operations or be a safety hazard;
(B) extend more than 6 ft 6 in (~198 cm) above the TILES; (C) communicate with anything or anyone outside the ARENA
(except medically required equipment); (D) block visibility for FIELD STAFF or the audience; (E) jam or interfere with
another team's remote sensing.
- Exceptions to B: MOMENTARY extensions, and people wearing PPE or reasonable decorative apparel (hats, headbands).
- Penalty: MATCH won't start until fixed; VERBAL WARNING if found or misused during a MATCH; YELLOW CARD for later
  violations at the event.
- Blue box: **Alignment devices for pre-MATCH setup are allowed** but must not delay the start (G301). Ladders and large
  signaling devices are hazard examples. Items with wireless turned off satisfy C. Jamming examples: **mimicking FIELD
  AprilTags**, or shining bright lights or lasers onto the FIELD during a MATCH.
- **[STRATEGY]** A physical alignment jig for repeatable auto start poses is legal.

**G303 \*ROBOTS on the FIELD must come ready to play.** Requirements at MATCH start: (A) no hazard to humans, ARENA, or
ROBOTS; (B) meets the 3.3 MATCH Eligibility inspection/re-inspection rules; (C) is the only team-provided item left on
the FIELD; (D) ROBOT SIGNS show the correct ALLIANCE color (R402).
- Penalty: MATCH won't start until fixed if quick; DISABLED if not quick, and Head REF may require re-inspection.
  **RED CARD if a ROBOT not compliant with G303.B plays in the MATCH.**
- Blue box: Head REF will likely consult the LRI.

**G304 \*ROBOTS must be set up correctly on the FIELD.** All of:
- A. fully on its own ALLIANCE side (**columns A, B, C = red; D, E, F = blue**, Fig 9-4);
- B. not attached to, entangled with, or suspended from any FIELD element;
- C. **touching the FIELD perimeter wall**;
- D. not touching or in the scoring volume of a FLOWER;
- E. not in the LOADING ZONE;
- F. within its STARTING CONFIGURATION (R102, R103);
- G. **in contact with exactly 4 POLLEN pre-loads** (10.3.1), fully on its own ALLIANCE side;
- H. **fully motionless once OpMode initialization finishes**.
- Penalty: MATCH won't start until fixed if quick; DISABLED or removed from the FIELD if not.
- Blue box: G304.A means fully inside the perimeter with no overhang of the wall. The 4 pre-load POLLEN may be in or on
  the ROBOT, or on TILES touching the ROBOT. They are allocated 4 per team. Touching other field-setup POLLEN doesn't
  count toward the pre-load. No-show ROBOTS have their POLLEN placed in the LOADING ZONE against the wall.
- Figure 11-1 (checked visually) shows legal starts at many points around the perimeter: corners, along side walls,
  rotated or diagonal ROBOTS, and odd shapes (triangle, circle, L, octagon). Every example touches the wall. The center
  structure is the two HIVES (red pair and blue pair).
- **[CODE]** INIT must leave the ROBOT fully still once init completes. No continuous servo hunting, no flywheel
  spin-up, no turret seeking in INIT. Auto start poses must be wall-touching, on our side, outside the LOADING ZONE, and
  clear of FLOWERS. Our start-pose table in code should only contain such poses.
- **[STRATEGY]** Starting position is flexible along any of our wall segments, so several auto start options are worth
  planning.
- Ambiguity: "touching the FIELD perimeter wall" is listed as a requirement, but the extracted text dropped "not" in
  the neighboring B item. The rendered page confirms C reads "touching", so a wall touch is required.

**G305 \*Teams must select an OpMode.** An OpMode must be selected on the DS and INITed. If it's an AUTO OpMode, **the
30-second AUTO timer must be enabled**.
- Penalty: MATCH won't start until fixed; DISABLED if the ROBOT can't init an OpMode or it can't be fixed quickly.
- Blue box: Every team must select and INIT an OpMode even with no auto planned. FIELD STAFF use this as the ready
  signal. Teams without an AUTO should make a default AUTO from the BasicOpMode sample and use auto-loading to queue
  TELEOP.
- **[CODE]** Every `@Autonomous` OpMode should set `preselectTeleOp = "..."` so TELEOP auto-queues. A trivial "do
  nothing" auto is a useful fallback.

### 11.4 In-MATCH (p104-115)

#### 11.4.1 AUTO (p104)
AUTO = **first 30 seconds**. DRIVERS give no input; the ROBOT runs only pre-programmed instructions. These rules apply
only during AUTO.

**G401 \*Don't interact with a ROBOT during AUTO.** From the start of the MATCH countdown to the end of AUTO, DRIVE TEAM
may not directly or indirectly touch the ROBOT or OPERATOR CONSOLE, except (A) pressing start (▶) within a MOMENTARY
margin of MATCH start; (B) pressing stop (■) at their discretion; (C) for personal or console safety.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC.
- Blue box: Teams need not start an OpMode if they're not running AUTO. G401.A is about starting on time with some
  human variability.
- **[CODE]** No gamepad input in AUTO, so the auto must be fully self-contained (alliance and start pose chosen during
  INIT, not after start). Pressing stop is always allowed as an abort.

**G402 No AUTO opponent interference.** During AUTO, a team may not disrupt the opposing ALLIANCE's AUTO.
- Penalty: **MAJOR FOUL per MATCH**; MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC.
- Blue box: In AUTO, columns A-C are red and D-F are blue. Each ALLIANCE has priority over FIELD and SCORING ELEMENTS
  on its side. **Going into the opponent's side in AUTO is risky and may be seen as STRATEGIC.** Disruption examples:
  (A) CONTROLLING SCORING ELEMENTS on the opponent's side; (B) touching an opposing ROBOT on its side. Elements that
  cross over after deflecting off something will likely not be penalized.
- **[CODE]** Auto paths must stay within our three columns, i.e. a centerline geofence (on a 144 in field, columns are
  24 in each, so red = x in [0,72], blue = [72,144] along the column axis; confirm axis orientation against Fig 9-4/9-5).
  Intake must not grab elements on the far side. Launching across into our target is fine; only CONTROL and contact on
  their side count.
- Ambiguity: the rule doesn't say whether a launched element that lands on the opponent's side (not deflected) counts
  as disruption. Avoid it.

#### 11.4.2 TELEOP (p104-105)

**G403 \*ROBOTS are motionless between AUTO and TELEOP.** No powered movement of the ROBOT or any MECHANISM during the
AUTO-to-TELEOP transition.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC.
- Blue box: Movement from inertia, gravity, or de-energizing actuators isn't powered movement. Teams may stop AUTO and
  INIT or start TELEOP on the DS during the transition. **If TELEOP's INIT moves or twitches anything, wait until TELEOP
  begins before pressing INIT.**
- **[CODE]** AUTO must stop all actuators at 30 s (the SDK timer stops the OpMode; make sure `stop()` and the end of
  the auto leave motors at zero power, flywheel off or coasting). TELEOP `init()` and `init_loop()` must command **no**
  motion: no servo position writes that move the turret or gate, no flywheel. A servo that is already holding position
  receiving the same position is fine, but a turret servo re-homing on INIT is a violation. Spinning the flywheel down
  by cutting power counts as "de-energizing" and is allowed.

**G404 \*ROBOTS are motionless at the end of TELEOP.** No powered movement after TELEOP ends until the Head REF or a
designee says teams may retrieve ROBOTS.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC.
- Blue box: Make it obvious by pressing stop (■) or stopping operation at the end of the MATCH, and setting down
  controllers. Inertia, gravity, and de-energizing don't count.
- **[CODE]** No post-match automated routines. A queued command that finishes after the buzzer (e.g. an auto-fire
  sequence) could violate this. Consider a match timer that cancels all commands at 2:00 (or let the DS stop the
  OpMode).

#### 11.4.3 SCORING ELEMENT (p105-108)

**G405 \*Keep SCORING ELEMENTS in bounds.** A ROBOT may not deliberately eject a SCORING ELEMENT from the FIELD
(directly or by bouncing it off a FIELD element or ROBOT).
- Penalty: **MAJOR FOUL per SCORING ELEMENT.**
- Blue box: Elements leaving the FIELD during scoring attempts or from ROBOT-to-ROBOT contact are not deliberate.
- **[CODE]** Don't program an "eject unwanted/opponent ball" routine that fires it out of the field. Dump to the floor
  inside the field instead.

**G406 \*Do not damage or make a mess in the ARENA.** Neither ROBOT nor DRIVE TEAM may make a mess or damage the ARENA.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC; DISABLED if the ROBOT causes damage and
  more is likely. Corrective action (remove sharp edges or the mechanism, re-inspection) may be required.
- Blue box: Aimed at harm needing significant repair or cleanup. See R201. Normal wear on elements (scratches, marks,
  fatigue) is expected. Routine gouging, tearing, or marking is a violation.

**G407 No more than 4 at a time.** A ROBOT may not CONTROL more than **4** SCORING ELEMENTS at once.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC.
- Blue box: Controlling 5+ at any time draws scrutiny. Likely STRATEGIC: (A) picking up and controlling 6+ and moving
  them to score; (B) multiple instances of more-than-MOMENTARY control of 5+. Likely not STRATEGIC: (C) MOMENTARILY
  controlling 5 then quickly reversing so at least one returns roughly to where it was. Teams are encouraged to add
  guards (nothing stuck on top) and systems that prevent intaking more than 4. **REPEATED 5+ control is likely
  STRATEGIC even with a "keep 4, discard the rest" sorter.** Not CONTROL: (D) bulldozing (incidental contact in the
  ROBOT's path); (E) deflecting (being hit by a bouncing element); (F) elements already LAUNCHED and no longer touching
  the ROBOT.
- **[CODE]** The intake subsystem needs an element counter (beam break or color/distance sensors) that **auto-stops or
  reverses the intake at 4**. This is critical: pre-loads are exactly 4, so the ROBOT starts full. The intake must stay
  off until at least one is launched. Launched balls stop counting once they leave the ROBOT, so the counter can drop
  on launch.
- **[STRATEGY]** Capacity is 4 per cycle.

**G408 Don't CONTROL opponent NECTAR.** A ROBOT may not CONTROL the opponent's NECTAR.
- Penalty: VERBAL WARNING; YELLOW CARD per MATCH if STRATEGIC.
- Blue box: REPEATED control of opponent NECTAR is likely STRATEGIC, even with a sorter that auto-discards it back to
  roughly its original state.
- **[CODE]** If intaking near NECTAR, use color sensing to reject opponent-color NECTAR **before** it is controlled
  (e.g. gate or intake reverse on detection at the mouth). Relying on discard-after-pickup is risky if it happens
  repeatedly. Best to avoid intaking where opponent NECTAR is.
- Note: no explicit rule bars controlling opponent POLLEN (POLLEN appears to be shared/neutral). Check Section 10 for
  POLLEN ownership.

**G409 Don't catch SCORING ELEMENTS.** A ROBOT may not catch or deflect a SCORING ELEMENT released by a TIPPED HIVE
until that element touches something other than that ROBOT.
- Penalty: VERBAL WARNING; YELLOW CARD per MATCH if STRATEGIC.
- Blue box: POLLEN and NECTAR spilling from a TIPPED HIVE should hit the TILE before any ROBOT collects it. Accidental
  catches by ROBOTS that happen to be under a tipping HIVE aren't the target. Guards against accidental catches are
  encouraged (they make it obvious catching isn't a strategy). Likely STRATEGIC: (A) REPEATEDLY parking under the HIVE
  waiting for a TIP; (B) a wide-opening top mechanism used to catch while driving under; (C) REPEATEDLY positioning so
  falling elements hit the ROBOT and get redirected advantageously. Likely not: (D) driving under and having 1-2 POLLEN
  land on a flat surface; (E) a dead ROBOT stationary under the HIVE collecting elements on top.
- **[CODE]** Don't put "wait under the HIVE" in auto or assist logic. After a tip, wait until elements settle on the
  floor before intaking.

**G410 NECTAR only goes into FLOWERS with one minute left.** ROBOTS may not put NECTAR into the FLOWER scoring volume
until the **last 60 seconds** of the MATCH.
- Penalty: **MAJOR FOUL per NECTAR.**
- Blue box: Scoring volume is defined in 10.5.2. **The primary FIELD timer display is the official cue for 60 s left.**
  Make it obvious and unambiguous to REFEREES that NECTAR isn't going in early.
- **[CODE]** Gate any automated NECTAR-into-FLOWER action on a match timer: TELEOP is 2:00, so 60 s left = 60 s into
  TELEOP (MATCH time 1:30 after the transition). Since the field display is authoritative and the robot timer may
  drift, add margin (e.g. lock until ~58 s left on the robot clock, i.e. 62 s into TELEOP) and a driver override or a
  gamepad rumble cue at the threshold. **Never in AUTO.**
- **[STRATEGY]** NECTAR-into-FLOWER is an endgame-only action. Plan last-minute cycles.

**G411 \*Do not hoard SCORING ELEMENTS.** An ALLIANCE may not STRATEGICALLY keep the opponent from accessing SCORING
ELEMENTS.
- Penalty: MAJOR FOUL + YELLOW CARD per MATCH.
- Blue box: Using your own NECTAR and/or POLLEN to score is fine. Corralling relevant elements into a limited area
  and/or positioning a ROBOT to block opponent access is a violation. **A GARDEN is not a protected zone.**

#### 11.4.4 ROBOT (p108-110)

**G412 \*ROBOTS must be under control.** A ROBOT must not pose an undue hazard to a human or ARENA element:
(A) the ROBOT or anything it CONTROLS disrupts anything outside the FIELD or touches a human outside the FIELD;
(B) operation is dangerous.
- Penalty: DISABLED + VERBAL WARNING; DISABLED + YELLOW CARD per MATCH for later violations at the event.
- Blue box: Watch out for REFEREES and FIELD STAFF nearby. Examples: (A) flailing wildly outside the FIELD;
  (B) knocking over a DS stand; (C) moving or damaging the FIELD timer display; (D) touching FIELD STAFF or DRIVE TEAM
  outside the FIELD. Touching ARENA elements outside the FIELD (DS stand, floor, outside of the wall) is not a
  violation by itself.
- **[CODE]** Launcher aim limits: the turret and flywheel must not be able to fire out of the field toward people.
  Add a soft limit or interlock so the robot only fires when a valid target solution exists.

**G413 \*ROBOTS must stop when instructed.** If a REFEREE tells a team to DISABLE (T402), a DRIVE TEAM member must press
stop (■) and set down the controllers.
- Penalty: VERBAL WARNING; RED CARD if STRATEGIC.

**G414 \*ROBOTS must be identifiable.** The ROBOT's team number and ALLIANCE color must not become indeterminate (Head
REF's view).
- Penalty: VERBAL WARNING; YELLOW CARD if STRATEGIC.
- Blue box: Attach ROBOT SIGNS robustly in visible spots. Revised signs may be required before the next MATCH.

**G415 \*Watch your ARENA interaction.** A ROBOT may not (A) grab, (B) grasp, (C) attach to, (D) become entangled with,
or (E) suspend from any ARENA element other than SCORING ELEMENTS.
- Penalty: VERBAL WARNING; YELLOW CARD per MATCH if STRATEGIC; DISABLED if damage is likely. Corrective action may be
  required.
- Blue box: A concave ROBOT shape that wraps partly around a FLOWER for alignment is legal.
- **[STRATEGY]** A concave alignment notch for FLOWERS is legal.

**G416 ROBOTS have construction limits.** During the MATCH, ROBOTS may not (A) expand past R105.A/B limits, or
(B) deliberately detach parts (R105.C).
- Penalty: VERBAL WARNING; MAJOR FOUL per instance if STRATEGIC.
- Blue box: Parts may extend beyond STARTING CONFIGURATION after the start, but must stay inside the expansion limit
  and not deliberately detach.

**G417 ROBOTS may not meddle with the HIVE structure.** A ROBOT may not affect HIVE motion in any way except
**LAUNCHING SCORING ELEMENTS into an upward-facing CELL**.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC.
- Blue box: TIPS should only come from launching POLLEN or NECTAR into an upward-facing CELL. Any other interaction
  that causes, could cause, or impedes a TIP is a violation. Likely STRATEGIC: (A) ramming the HIVE frame at high
  speed; (B) ramming it repeatedly in a short time; (C) deliberately launching into the external bottom, sides, or top
  faces of a CELL; (D) **impeding the TIP of an opponent's HIVE by launching at it**; (E) touching a HIVE directly or
  through a CONTROLLED element; (F) repeating after a warning. Likely not: (G) accidentally bumping the frame while
  picking up POLLEN; (H) attempting to launch into the upward-facing CELL and missing so it hits the bottom, sides, or
  top.
- **[CODE]** Turret targeting should only aim at **upward-facing** CELLS of our HIVE. If `LaunchTargets` selects the
  nearest CELL, it must know which CELLS currently face up (HIVE orientation changes after a TIP). Aiming at a
  non-upward CELL, or at the opponent's HIVE to block their tip, risks G417. Path planning should keep clearance from
  the HIVE frame.
- Ambiguity: "deliberately launching into external faces" vs. "missing" depends on referee-perceived intent. Aiming
  consistently at the opening and shooting only when `isReadyToLaunch()` is true helps show intent.

**G418 ROBOTS may not meddle with FLOWERS.** ROBOTS may put elements into or take them out of a FLOWER only by:
(A) entering POLLEN and NECTAR into the **top**; (B) removing **POLLEN only** from the **bottom**.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC.
- Blue box: Entry is through the top of the top ring only. Only POLLEN (not NECTAR) may be removed, from the bottom of
  the middle ring. Touching a FLOWER is fine. Not violations: (A) touching the FLOWER while entering or removing;
  (B) touching scored elements while scoring; (C) driving into a FLOWER and touching elements inside. Likely STRATEGIC:
  (D) pulling POLLEN through the side; (E) grabbing and shaking the FLOWER; (F) forcing NECTAR out of the middle ring;
  (G) deliberately ramming the perimeter wall at high speed to knock elements out of a FLOWER; (H) holding NECTAR
  against the side of the FLOWER pipes so it meets scoring criteria. Likely not: (I) knocking POLLEN out the top while
  picking up POLLEN from the TILES; (J) knocking out a scored POLLEN while scoring another.
- **[STRATEGY]** FLOWERS are a POLLEN source (bottom removal).

#### 11.4.5 Opponent Interaction (p110-112)
Note: G419 and G420 are mutually exclusive. One ROBOT-to-ROBOT interaction that breaks both gets **only the most
punitive** penalty.

**G419 \*Do not damage an opponent ROBOT.** No damaging or functionally impairing an opponent ROBOT.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per instance if STRATEGIC; **MAJOR FOUL + RED CARD per instance if
  STRATEGIC and the opponent can't drive.**
- Blue box: High-contact and defense are allowed, so build robust. Damage to exposed or unprotected parts in normal
  play likely isn't a violation: (A) an exposed main power switch (R603); (B) exposed wiring; (C) delicate mechanisms
  such as an intake. Avoid damage obviously when an opponent is precarious. STRATEGIC example: (D) high-speed ramming
  and/or repeated smashing that causes damage. Functional impairment examples: (E) disconnecting internal wires;
  (F) disconnecting the battery (also "unable to drive"); (G) switching off a reasonably well-protected power switch
  (also "unable to drive"). Head REF may inspect after the MATCH and remove the violation if damage can't be verified.
  "Unable to drive" = can't reach a desired location in reasonable time (e.g. only circles, or extremely slow).
- **[STRATEGY]** Protect your power switch, wiring, and intake. Damage to unprotected parts is on you.

**G420 \*Do not tip or entangle.** No attaching to, tipping over, or entangling an opponent ROBOT.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per instance if STRATEGIC; MAJOR FOUL + RED CARD per instance if
  STRATEGIC and either CONTINUOUS or the opponent can't drive.
- Blue box: Entanglement with exposed wiring or delicate mechanisms in normal play likely isn't a violation. Likely
  STRATEGIC: (C) using a wedge to tip; (D) frame-to-frame contact with a ROBOT trying to right itself, making it fall
  again; (E) avoidable contact after a ROBOT starts tipping that tips it over. Tips from normal interaction, including
  a single frame-to-frame hit, are not STRATEGIC. Same "unable to drive" definition.

**G421 \*There is a 3-count on PINS.** No PINNING an opponent for more than **3 seconds**. PINNING = preventing an
opponent's movement by contact, direct or transitive (e.g. against a FIELD element). The PIN count ends when:
(A) the ROBOTS have been at least **2 ft (~61 cm)** apart for more than 3 s; (B) either ROBOT has been 2 ft from where
the PIN started for more than 3 s; or (C) the PINNING ROBOT gets PINNED.
- For A: the count pauses once they're 2 ft apart, until the PIN ends or the pinner moves back within 2 ft (then it
  resumes).
- For B: the count pauses once either has moved 2 ft from the start point, until the PIN ends or both move back within
  2 ft (then it resumes).
- Penalty: **MAJOR FOUL per instance + an additional MAJOR FOUL for every 3 seconds** it isn't corrected.
- **[STRATEGY]** Defense is legal with a 3-second limit, and a pin must break by 2 ft for more than 3 s to reset.
  Pinning resumes the count if you come back.

#### 11.4.6 Human (p112-115)

**G422 \*DRIVE TEAM stays in the ALLIANCE AREA.** After MATCH start, DRIVE TEAM may not leave their ALLIANCE AREA.
- Penalty: VERBAL WARNING; MINOR FOUL per instance if STRATEGIC.
- Blue box: Meant to stop moving for a better view or reaching into the FIELD. Breaking the plane of the AREA during
  normal play is fine. DRIVE TEAM may be anywhere in their AREA and **may retrieve their own ALLIANCE's NECTAR that
  left the FIELD if reachable from the AREA**. Safety exceptions apply.

**G423 \*DRIVE COACHES and other teams: hands off the controls.** After MATCH start, DRIVE COACHES and members of other
teams may not handle the OPERATOR CONSOLE gamepads.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC. The penalty goes to the team the violator
  belongs to.
- Blue box: Ask FIELD STAFF about accommodations in major-conflict cases. DRIVE COACHES may: (A) hold the DS device;
  (B) troubleshoot it; (C) select OpModes; (D) press INIT; (E) press start (▶); (F) press stop (■).
- **[CODE]** Only DRIVERS use gamepads, so all controls the coach needs must be on the DS screen (OpMode selection).
  Auto configuration (alliance, start pose) should be selectable by the drivers' gamepads in `init_loop` or by
  separate OpModes.

**G424 \*DRIVE COACHES, SCORING ELEMENTS are off limits.** Coaches may not touch SCORING ELEMENTS except for safety.
- Penalty: VERBAL WARNING; MINOR FOUL per instance if STRATEGIC.
- **[STRATEGY]** Only DRIVERS (not the coach) can do the human-player NECTAR feeding.

**G425 \*DRIVE TEAMS, watch your reach.** After MATCH start, DRIVE TEAM members may not directly or indirectly:
(A) touch a ROBOT; (B) touch a SCORING ELEMENT that is touching a ROBOT or TILE; (C) disrupt scoring; (D) touch a FIELD
element.
- Penalty: VERBAL WARNING; MAJOR FOUL + YELLOW CARD per MATCH if STRATEGIC.
- Blue box: Controllers, hanging wires, clothing, etc. in the AREA can cause indirect contact. Safety exceptions apply.
  For A and B, the penalty goes to the team the DRIVE TEAM member belongs to, regardless of who started the contact.
  Disrupting scoring includes: (A) blocking a ROBOT from scoring in a FLOWER; (B) deliberately hitting or shaking the
  wall to descore NECTAR or POLLEN from a FLOWER.
- Ambiguity: the penalty-assignment sentence is garbled in the source text ("applied to the team to which the DRIVE
  TEAM member regardless of whether the DRIVE TEAM member belongs or ROBOT initiates contact"). It most likely means
  "the team the member belongs to, regardless of whether the member or the ROBOT initiates contact."
- **[STRATEGY]** The human player must not touch our ROBOT in the LOADING ZONE, even if the ROBOT drives into their
  hand.

**G426 Humans may not enter NECTAR onto the FIELD early.** DRIVE TEAM may enter NECTAR only:
(A) **one additional NECTAR each time their own-color HIVE is TIPPED**, or (B) **all remaining NECTAR once 60 s or less
remain**, whichever comes first.
- Penalty: **MINOR FOUL per NECTAR.**
- Blue box: The primary FIELD timer is the 60-s cue. NECTAR that leaves the FIELD may be re-entered any time (G427).
  Keep remaining NECTAR visible and make legality obvious. NECTAR doesn't have to go in immediately after a TIP.
- **[STRATEGY]** Each tip of our HIVE earns 1 NECTAR to bring in, which links tipping to NECTAR supply. Banking NECTAR
  for later is allowed.
- **[CODE]** If we build a driver aid, count our HIVE tips (detected by vision or driver input) and show "NECTAR
  available" on the DS telemetry, plus a 60-s-remaining rumble.
- Ambiguity: "whichever comes first" is odd, since A and B are cumulative allowances. It likely means that once 60 s
  remain, all NECTAR may enter regardless of tip count.

**G427 Humans, only enter NECTAR via the LOADING ZONE.** NECTAR may be introduced only: (A) without a tool; (B) by a
DRIVE TEAM member of the matching ALLIANCE color; (C) so it **touches the TILE inside the LOADING ZONE before touching
a ROBOT or FIELD element**.
- Penalty: MINOR FOUL per NECTAR.
- Blue box: Only your own NECTAR into the LOADING ZONE. Don't push the boundaries. **Humans scoring into a HIVE or
  FLOWER is never allowed** (G202). POLLEN or opponent NECTAR that leaves the FIELD goes to FIELD STAFF for
  reintroduction (10.8). While introducing NECTAR, don't touch an element that is touching a TILE (G425).
- **[CODE]/[STRATEGY]** A ROBOT can't take NECTAR straight from a human's hand. It must hit the tile first. Any intake
  logic waiting in the LOADING ZONE must let it land.

**G428 Humans, don't remove SCORING ELEMENTS from the FIELD.** DRIVE TEAM may not remove SCORING ELEMENTS from the
FIELD.
- Penalty: MINOR FOUL per SCORING ELEMENT.
- Blue box: An element is on the FIELD if it touches a TILE, or is in or on a ROBOT, FIELD element, or another
  SCORING ELEMENT.

---

## 12 ROBOT Construction Rules (R) (p116-130)

### Intro and definitions (p116-118)
- ROBOT = electromechanical assembly built by an FTC team for the current game, with power, comms, control, and
  movement. All energy sources and actuation (batteries, motors, servos, controllers) come from a defined set. These
  rules cover the ROBOT as inspected; in-match consequences are in Section 11.
- **COMPONENT** = most basic part, can't be disassembled without damage or a change of function. **MECHANISM** =
  assembly of COMPONENTS that can be taken apart and reassembled without damage.
- **COTS** = standard (not custom-order) part commonly available from a VENDOR to all teams, unaltered (**software
  installation or modification allowed**). Discontinued but functionally equivalent items still count. Examples:
  1) a panel with lightening holes drilled = FABRICATED, the untouched twin = COTS; 2) a shop-made copy of a COTS
  design = not COTS; 3) published drawings = COTS "raw material", the built gearbox = FABRICATED; 4) non-functional
  labels keep COTS status, added mounting holes make it FABRICATED; 5) a discontinued COTS gearbox in original
  condition is OK.
- **VENDOR** criteria: (A) Federal Tax ID or foreign equivalent business registration; (B) not a wholly owned
  subsidiary of a FIRST team or teams; (C) enough stock or capacity to ship general items promptly (surge backorders
  excused; e.g. custom-welded belt treads with a 2-week lead time = FABRICATED item, OK; a plain belt shipped from
  stock within 5 business days = COTS); (D) available to all FTC teams.
- **FABRICATED ITEM** = anything altered, built, cut, machined, painted, coated, etc. into its final form. Raw stock
  cut only for storage is neither COTS nor FABRICATED.
- Teams may need to show documentation (cite rules) for limited parts (motors, servos, current limits, COTS
  electronics). Legality questions: customerservice@firstinspires.org.

### 12.1 General ROBOT Design (p118-122)
High-contact game, so build robust.

**R101 \*It is your team's ROBOT.** The ROBOT and its MAJOR MECHANISMS must be built by the registered team. MAJOR
MECHANISM = group of COMPONENTS/MECHANISMS handling at least 1 game challenge (movement, SCORING ELEMENT manipulation,
FIELD element manipulation, scorable task).
- Blue box: Help from other teams (fabrication, construction, **writing software**, strategy, parts) is not
  prohibited. Not MAJOR MECHANISMS: gearbox assemblies, sub-parts of a MAJOR MECHANISM, COTS items. Intent: the ROBOT
  reflects the current team's experience; no turnkey outside solutions. See R301, R303.

**R102 \*STARTING CONFIGURATION is limited to an 18-inch cube.** At start, all parts **fully stationary** and the
ROBOT within **18 in x 18 in x 18 in (45.70 cm)**.
- Blue box: With interchangeable mechanisms (3.3), show R102/R105 compliance in every configuration. **Pre-loaded
  SCORING ELEMENTS may stick out past the 18-in limit.**

**R103 \*ROBOTS may assist in holding the STARTING CONFIGURATION.** The ROBOT must be self-supported (no force on
the sizing tool's sides or top), by (A) mechanical means while powered off and/or (B) **initializing an OpMode that
pre-positions servos and motors to a stationary position**. The OpMode may hold motors and servos in position to keep
the STARTING CONFIGURATION.
- Blue box: Holding may last several minutes (inspection or queuing), so limit thermal risk (no motors stalled
  against hard stops). Tell the INSPECTOR when the ROBOT is live.
- **[CODE]** INIT may move servos to a stowed pose (allowed pre-match). Combined with G304.H, the movement must finish
  and then stay still. In TELEOP INIT after AUTO, G403 forbids that motion. For our 5-turn turret servo, a stow
  position must keep the robot within 18 in. Avoid stalled motor holds; use low or zero power holds and brake mode.

**R104 \*There is no ROBOT weight limit.**
- Blue box: Still consider tile damage, battery use, transport, and performance.

**R105 A ROBOT must stay one assembly, with expansion limits.** After the start:
- A. may expand but must always fit within **18 in (45.70 cm) x 24 in (61.0 cm) x 29 in (73.65 cm) tall** (per G416);
- B. **must be physically constrained to these limits without software**;
- C. may not be designed to deliberately detach COMPONENTS.
- The 29-in dimension is always vertical above the FIELD surface (Fig 12-1 top view, 12-2 side view).
- Blue box: Measured in stable gameplay configurations (wheels on TILES); slight tilt from contact is OK if it would
  fit when level. **Show maximum mechanical extensions at inspection. A mechanism that can mechanically exceed the
  limit is illegal even if software limits it** (Fig 12-3). A single mechanism extending out both sides is OK if its
  total dimension at max extension fits (12-4). Multiple non-linked mechanisms that can extend out both sides at once
  are NOT OK if the total horizontal dimension exceeds the limit (12-5). A horizontal pivoting extension must fit at
  every point of travel (12-6).
- **[CODE]** Software limits on extension do NOT satisfy R105. Hard stops are required. Software limits are still fine
  as an extra layer.
- Ambiguity: which horizontal axis gets 18 in vs 24 in is defined relative to the starting configuration (Fig 12-1).
  Text extraction didn't capture the figure detail.

### 12.2 Fair Play & Damage Prevention (p122-123)

**R201 \*Design ROBOTS to not damage or make a mess.** No risk of mess or hazard in the ARENA.
- Blue box: Normal wear on elements is OK. Gouging, tearing, or routine marking violates this and G406. Damage-risk
  examples: (A) traction devices known to damage TILES; (B) exposed sharp edges or protrusions; (C) abrasive surfaces.
  Mess-risk examples: (D) excess lubricant; (E) loose parts or ballast (sand, coffee beans, kitty litter, glitter, ball
  bearings); (F) liquids or gels; (G) tire sealant; (H) other lubricants including graphite powder.

**R202 \*Design ROBOTS and OPERATOR CONSOLES for safety and fair play.** No hazardous materials, unsafe conditions,
or interference with other ROBOTS or FIELD STAFF. Examples that may violate: (A) shields or curtains to block DRIVE
TEAM vision; (B) loud audio that distracts or mimics MATCH sounds; (C) devices meant to interfere with other ROBOTS'
sensing (vision, ultrasonic, sonar, IR), **including imagery that uses or closely mimics 36h11 AprilTags**;
(D) flammable gas, flames, or pyrotechnics; (E) mercury, lead, etc.; (F) **high-intensity lights should only be on
briefly while targeting** and may be scrutinized, and the Head REF or LRI may require disabling or modifying them;
(G) animal-based materials; (H) devices to damage or flip ROBOTS; (I) unnecessary entanglement risk; (J) **lighting
flashing faster than 5 Hz** invites scrutiny and may need to be disabled or modified; (K) other violations of the
spirit.
- **[CODE]** Limelight LEDs: enable only while actively targeting (e.g. pipeline LED mode on only during
  relocalization or aim), not continuously. Indicator lights must not blink faster than 5 Hz. Check any flashing
  patterns in the alignment-indicator code.

**R203 \*ROBOTS must be removable from the FIELD without power.** SCORING ELEMENTS must come out of the ROBOT, and the
ROBOT off FIELD elements, while powered off.
- Blue box: Some events allow driving during reset, but don't rely on it.
- **[CODE]/design** Servo-gated launcher storage must allow manual removal with power off (servo backdrivable or
  open path).

**R204 \*No grabbing the floor.** No mechanism designed to increase downforce by grabbing FIELD surfaces or by
airflow suction.

### 12.3 Fabrication (p124-125)

**R301 \*COTS MECHANISMS are legal but limited.** COTS MAJOR MECHANISMS purpose-built for a game task are banned.
Exceptions: (A) COTS drive CHASSIS (all parts legal); (B) COTS MAJOR MECHANISMS from the official FTC StarterBots.
- Blue box: COTS parts help build mechanisms but shouldn't be turnkey game solutions. A vendor offering "build to print"
  of public purpose-built designs is against the spirit.

**R302 \*Legal COTS parts and raw materials may be modified** (drilled, cut, painted...) if no other rule is broken.
Raw materials include: (A) sheet stock; (B) extrusions; (C) metal, plastic, rubber, wood; (D) magnets.

**R303 \*COTS must be single DoF.** COTS COMPONENTS and MECHANISMS may not exceed 1 mechanical degree of freedom.
Allowed single-DoF examples: (A) linear slide kit; (B) linear actuator kit; (C) single-speed (non-shifting) gearbox;
(D) pulley; (E) turntable; (F) lead screw; (G) single-DoF gripper. Exceptions: (H) ratcheting devices; (I) holonomic
wheels (omni, mecanum); (J) **dead-wheel odometry kits**; (K) motion transfer between misaligned parts (U-joints,
flexible couplers); (L) variable-angle connectors (ball joints, rod ends).
- Blue box: Also bans specialized parts that only assemble into a multi-DoF COTS item. Test: can every part's pose be
  predicted from one input? Ex 1: a mecanum drivetrain of 4 single-DoF modules is single DoF. Ex 2: dead-wheel modules
  (wheel + spring) are 2 DoF and explicitly allowed. Ex 3: single- or dual-actuator grippers are single DoF, but
  wrist-type twist/bend adds DoF and is banned in COTS.
- Note: our goBILDA Pinpoint with dead-wheel pods is fine under J.

**R304 \*Custom software, designs, and parts can be reused year to year.** Software, designs, and FABRICATED ITEMS
created before Kickoff are allowed.
- **[CODE]** Reusing last season's code (subsystems, Pedro setup) is legal.

**R305 \*SCORING ELEMENTS are not allowed for ROBOT construction.** Current-season SCORING ELEMENTS or replicas may not
be part of the ROBOT.

### 12.4 ROBOT SIGN Rules (p126-128)
Purpose: field staff identify team number and ALLIANCE. Templates in US Letter and A4.

**R401 \*Minimum of two ROBOT SIGNS per ROBOT**, on opposite or adjacent surfaces at least 90 degrees apart (the top
counts). Each sign: (A) robust material; (B) at least **6.5 in (16.5 cm) wide**; (C) at least **2.5 in (6.4 cm) tall**
(Fig 12-7); (D) supported by the ROBOT frame.
- Blue box: Readable from at least 12 ft (3.65 m) before, during, and after the MATCH. Robustness = good-faith effort.

**R402 \*ROBOT SIGNS indicate your ALLIANCE.** Each sign has a solid opaque red or blue rectangle at least
**6.5 x 2.5 in (16.50 x 6.35 cm)** matching the scheduled ALLIANCE. Other visible markings are banned except:
(A) R403 numbers; (B) small hook-and-loop, fasteners, or equivalents; (C) narrow areas of different color at corners,
folds, or cutouts; (D) narrow template markings. (E) The sign may not use power to light up or reveal ALLIANCE color.
Reversible or configurable signs must not show the opposite color except as allowed. (Fig 12-8)
- **[CODE]** You cannot use LEDs or displays to show alliance color on signs.

**R403 \*Team number on ROBOT SIGNS** (Fig 12-7, 12-9, 12-10): (A) solid opaque **white** Arabic numerals about
**2.25 in (5.70 cm)** tall; (B) about **0.25 in (0.60 cm)** background margin around them; (C) **no vertical
stacking**; (D) robust materials; (E) not powered or illuminated.
- Blue box: If legal signs can't be made at the event, the Head REF may approve a substitute (even handwritten on
  paper). Small decorations or gaps are OK if readable at 12 ft at a glance. Prohibited: edge-lit engraved plastic
  numbers, LED displays.

### 12.5 Motors & Actuators (p129-130; continues past p130)

**R501 \*Only specific motors are allowed.** Table 12-1:

| Motor | Part Numbers | Notes |
|---|---|---|
| AndyMark NeveRest 12V DC | am-3104, am-3104b | |
| AndyMark NeveRest Hex 12V DC | am-3104c | |
| goBILDA Yellow Jacket 520x Series 12V DC | 5201-0002-0026, etc. | 5201, 5202, 5203, 5204 series |
| goBILDA 5000 Series 12V DC | 5000-0002-4008, etc. | |
| Modern Robotics / MATRIX 12V DC | 5000-0002-0001 | Discontinued |
| NFR Products Yuksel 12V DC | NFR-600-100-000 | |
| REV Robotics HD Hex 12V DC | REV-41-1291 | |
| REV Robotics Core Hex 12V DC | REV-41-1300 | |
| Studica Robotics Maverick 12V DC | 75001 | |
| SWYFT Robotics SWYFT Spike Motor | SR-MOTOR-DC-01 | |
| TETRIX MAX 12V DC | 739530, 39530 | Discontinued |
| TETRIX MAX TorqueNADO 12V DC | W44260 | |
| WATTOS Stingray 12V DC | WDM12 | |

- Also allowed: factory vibration and autofocus motors built into COTS computing devices (as part of the device only),
  and motors built into unmodified COTS sensors (LIDAR, scanning sonar). **Neither counts toward the R503 limit**
  (R503 is past p130).
- Blue box: More motors may be added in later updates. Gearmotors may be used with or without their gearbox, or with
  any compatible gearbox.
- Note: our goBILDA BARE motors (5202/5203-series, no gearbox) are legal flywheel motors.

**R502 \*Servo usage is restricted.** Servos must be compatible with their power source (R505) and may include extra
position-feedback outputs (e.g. 4th-wire feedback). Table 12-2 (requirements at 6V):

| Class | Mechanical Output Power | Stall Current | Examples |
|---|---|---|---|
| Servo | **≤ 8 W @ 6V** | **≤ 4 A @ 6V** | AndyMark High-Torque (am-4954), Axon MAX+, DSSERVO 35KG Coreless (DS3235MG), FEETECH FT5335M-FB, goBILDA Dual Mode (2000-0025-0003), REV Smart Servo (REV-41-1097), Studica Multi-Mode Smart Servo (75002) |
| Linear Servo | N/A | **≤ 1 A @ 6V** | Actuonix Micro Linear Servo (P8-100-252-12-R), Hitec Linear Servo (HLS12-3050-6V), Studica Linear Servo RC Actuator (75014) |

- Formula: **Mechanical Output Power = 0.25 x Stall Torque (N·m) x No-Load Speed (rad/s)**, using the maker's 6V data.
- Blue box: Both limits must be met. Pre-approved list is in the Inspection Quick Reference; otherwise bring spec
  documentation; an online calculator is available. If no 6V specs exist, specs at higher voltages may be used. Stall
  current = maximum possible, **ignoring any software-adjustable current limits**. Power: **REV Control Hub and
  Expansion Hub supply 5V to servos**; goBILDA Servo Power Injector, REV Servo Power Module, Studica Servo Power Block,
  and REV Servo Hub supply 6V. Servos rated 6-8.4 V may misbehave at 5V.
- **[CODE]/design** Check that our 5-turn turret servo (likely goBILDA Super Speed/Torque 5-turn or similar) meets
  ≤ 8 W / ≤ 4 A, and is powered at the voltage it needs. A 5V hub port could change speed and timing, which affects
  turret settle-time tuning in `isReadyToLaunch()`.

---

## Ambiguities and open questions (summary)
1. G304.C "touching the FIELD perimeter wall" is confirmed as a requirement by the rendered page; the text extraction
   was garbled here.
2. G402: unclear whether a launched element landing on the opponent's side without deflection is "disruption".
3. G410 and G426 use the **FIELD timer display** as the authority for 60 s left. Robot-side timers need margin.
4. G417: the line between a "missed shot" and "deliberately launching into external faces" is referee judgment.
   Which CELLS face upward changes after TIPS, so targeting code must track it.
5. G425 penalty-assignment sentence is garbled; best reading is "penalty to the member's team regardless of who
   initiated contact."
6. G426 "whichever comes first" wording is odd for cumulative allowances.
7. R105: which horizontal axis is 18 vs 24 in is shown only in figures 12-1/12-2.
8. G408 covers only opponent NECTAR; POLLEN ownership and control rules need checking in Section 10.
9. R503 (motor count limit) and power rules (R505+) are beyond p130.

---

# BIOBUZZ Manual (TU07) Notes, Part 4: Pages 131-171

Covers: the end of Section 12 ROBOT Construction (from R503 onward: 12.5 actuators (tail), 12.6 Power Distribution, 12.7 Control/Command/Signals, 12.8 Pneumatics, 12.9 OPERATOR CONSOLE), Section 13 Tournament (T), Section 14 League Play (L), Section 15 FIRST Championship (C), and Section 16 Glossary.

Legend: **[CODE]** = constrains robot software or hardware config. **[STRATEGY]** = matters for match or event strategy. "Box:" = a shaded clarification box. These boxes render orange/salmon in this PDF; the task called them blue boxes. Every rule title starts with "*" in the manual. R501/R502 (the allowed motor and servo lists) are on page 130 and fall outside this range; see notes_3.

Page 164 is blank.

---

## 12.5 Actuators (continued) (p131)

### R503 *ROBOTS are limited to a total of 8 motors and 8 servos.* **[CODE]**
- At most 8 motors and 8 servos from the R501/R502 lists, counted across all MECHANISMS in all configurations.
- Box: if a ROBOT uses multiple configurations with different MECHANISMS at one event, the **sum** of all motors and servos across those configurations must stay within the limit. Swapping modules doesn't reset the count.
- Code impact: the hardware map / RC configuration can have at most 8 motors and 8 servos in total. Today the team uses 2 flywheel motors, drive (likely 4), intake, a turret servo, the launch gate, and indicator-light servos.
- **Ambiguity:** the indicator "lights" are driven by servo-PWM. If the light device is a servo-PWM LED (e.g. goBILDA RGB indicator), whether it counts toward the 8-servo limit depends on R502 and the light's classification (LEDs vs. servo). Check R502 (p130) and Q&A. If they count, they use up the servo budget.

### R504 *Do not modify actuators unless explicitly allowed.*
The integral mechanical and electrical system of any motor or servo must not be modified, except:
- A. mounting brackets and/or output shaft/interface (including pinion gears) may be modified to attach the motor to the ROBOT and the actuated part;
- B. electrical leads may be trimmed, and connectors or splices added (per R609). Purely electrical enclosures may be replaced with functionally equivalent ones;
- C. servos may be modified as the manufacturer specifies (e.g. setting soft limits, continuous-rotation modification). **[CODE]** This means servo programmers that set soft limits are allowed;
- D. labeling is allowed if it doesn't obstruct identifying markings;
- E. insulation may be applied to electrical terminals;
- F. repairs are allowed if original performance and specs are unchanged;
- G. manufacturer-recommended maintenance is allowed.

### R505 *All actuators must be controlled and powered through approved devices.* **[CODE]**
All actuator control signals must originate from a power regulating device. Exception: servos, fans, or motors integral to sensors of COTS computing devices permitted in R501. The only allowed power regulating devices (Table 12-3) are:

| Power Regulating Device | Part Number | Load Limit |
|---|---|---|
| goBILDA 6V Servo Power Injector | 3125-0001-0001 | 2 servos per port |
| REV Control Hub / Expansion Hub **Motor** Ports | REV-31-1153 / REV-31-1595 | 2 motors per port |
| REV Control Hub / Expansion Hub **Servo** Ports | REV-31-1153 / REV-31-1595 | 2 servos per port |
| REV Servo Power Module | REV-11-1144 | 2 servos per port |
| REV Robotics Servo Hub | REV-11-1855 | 2 servos per port |
| REV SPARKmini | REV-31-1230 | 2 motors per device |
| Studica Servo Power Block | 75005 | 2 servos per port |

- Note: up to 2 motors per hub motor port (Y-split) is legal. In software, both motors on one port share a single channel, so they can't be controlled independently.

### R506 *The use of relays, electromagnets, and electrical solenoid actuators is prohibited.*
Electromechanical actuation through additional relays, electromagnets, electrical solenoid actuators, or related systems is prohibited, and so is any use of relays or electromagnets.

---

## 12.6 Power Distribution (p132-136)

### R601 *ROBOTS can contain only 1 main battery.*
- Exactly 1 approved 12V NiMH main battery. It is the only legal energy source for the control system and actuation, and it must stay unaltered except:
  - A. the fuse may be replaced with a COTS equivalent in-line **20A ATM mini blade fuse** (installed per R610, the manual's cross-reference);
  - B. connectors may be replaced with popular connectors of comparable rating (Anderson Powerpole, XT30, etc.).
- Legal main batteries (Table 12-4):

| Battery Pack | Part Number | Notes |
|---|---|---|
| AndyMark Flat Pack Battery DC 12V | am-5290 | |
| goBILDA 12V NiMH Nested Battery | 3100-0012-0020 | |
| Matrix 12V 3000mAh NiMH | 14-0014 | May be labeled "Modern Robotics" |
| REV 12V Slim Battery | REV-31-1302 | |
| Studica 12V 3000mAh NiMH | 70025 | |
| TETRIX MAX 12V 3000mAh NiMH | W39057 | Formerly 739023 |
| WATTOS 12V Battery | WT-NMH1230 | |

- Box: similar batteries from other vendors are NOT legal. Only the listed manufacturers and part numbers are. Other batteries are allowed only per R602.

### R602 *Other batteries are only allowed for self-contained peripheral devices and LEDs only.*
- COTS USB battery packs of **at most 100Wh (27,000mAh at 3.7V)** and **at most 5V/5A, or 12V/5A via USB-PD, per port** are allowed. So are batteries integral to self-contained devices (e.g. a GoPro-style camera). Conditions:
  - A. they must not power any ROBOT actuators, and
  - B. they must not be used by any device that receives control signals from the ROBOT control system (USB packs must stay electrically isolated from ROBOT power).
- Exceptions to B: (i) powered USB hubs; (ii) ROBOT CONTROLLER smartphones.
- Box: **any device receiving signals from a REV Control or Expansion Hub must be powered by the main ROBOT battery.** **[CODE]** For example, a Limelight on USB is powered through the hub's USB. Don't power sensors from a separate pack.

### R603 *Connect the ROBOT battery through the Main Power Switch.*
Exactly one main power switch must control all battery power to all power regulating devices (except R602 items).
- A. It must be an approved switch (Table 12-5):

| Power Switch | Part Number |
|---|---|
| AndyMark FTC Power Switch w/ Bracket | am-4969 |
| goBILDA Floodgate Power Switch | 3103-0005-0001 |
| REV Switch Cable and Bracket | REV-31-1387 |
| Studica On/Off Power Switch Kit | 70182 |
| TETRIX R/C Switch Kit | W39129 |
| WATTOS Power Switch Kit | WTS-SW1220 |

- B. It must be accessible to the team and away from high-speed moving parts and pinch hazards.
- C. Secondary power switches may be used on the 12V line downstream of the main switch.
- Box: there are no specific location requirements, but keep it clear of moving parts and pinch points. It may sit behind removable panels. It should be protected from robot-to-robot contact so it can't be bumped off or damaged. **[STRATEGY]** A switch knocked off mid-match ends your match.

### R604 *Fuses must be used as directed and must not be modified.*
Use fuses as the device manufacturer directs. Don't modify them, use a higher trip point, or substitute self-resetting fuses.

### R605 *The ROBOT frame cannot be used as a current path.*
All wiring and electrical devices must be electrically isolated from the frame, and the frame must not carry current. Grounding the control electronics to the frame is allowed only if all of these hold:
- A. the strap/cable is an approved part (Table 12-6): AndyMark Resistive Grounding Strap am-4648a; REV Resistive Grounding Strap REV-31-1269; Swyft Grounding Cable SR-Ground-01;
- B. the strap connects directly to a fully COTS COMPONENT with an XT30 connector and directly to the frame (via the resistive terminal);
- C. no COMPONENTS or MECHANISMS are designed to ground the ROBOT frame to the FIELD.
- Box: see the ROBOT Wiring Guide for strap installation. (A grounding strap reduces ESD disconnects. Recommended.)

### R606 *ROBOT CONTROLLER and the ROBOT electrical system must be inspectable.*
- A. All power regulating devices (R505), their wiring, and all fuses must be able to be made visible for inspection.
- B. The ROBOT CONTROLLER must be mounted so its diagnostic lights (or screen) can be seen for inspection.
- Box: "visible for inspection" doesn't require visibility in STARTING CONFIGURATION or during a MATCH, as long as the team can expose the items during inspection. Teams are strongly encouraged to keep diagnostic lights visible in all MATCH configurations. If they aren't, FIELD STAFF may not be able to fully support the team (e.g. diagnosing disconnects). **[STRATEGY]**

### R607 *Custom Circuits must not provide regulated power above 5V unless they are only powering LEDs.*
A CUSTOM CIRCUIT is any active electrical item that isn't an actuator (R501) or a power regulation device (R505). CUSTOM CIRCUITS may not output regulated voltage above 5V unless it powers LEDs only. They may pass through unregulated battery voltage.

### R608 *All Power regulating devices must be connected and powered through approved ports.* (Table 12-7)

| Device | Part Number | Method of Powering |
|---|---|---|
| goBILDA 6V Servo Power Injector | 3125-0001-0001 | Only via its XT30 connectors, from the main battery |
| REV Control Hub / Expansion Hub | REV-31-1153 / REV-31-1595 | Only via its XT30 connectors, from the main battery |
| REV Servo Power Module | REV-11-1144 | Only via screw terminals, main battery only |
| REV Robotics Servo Hub | REV-11-1855 | Only via power terminals, main battery only |
| REV SPARKmini | REV-31-1230 | Only via Power input, main battery only |
| Studica Servo Power Block | 75005 | Only via JST-VH power connector, main battery only |

All of them must also be powered per the manufacturer's instructions.

### R609 *Use appropriately sized wire.* (Table 12-8)

| Application | Minimum Wire Size |
|---|---|
| 12V main battery power; motor power (unless listed below); 11-20A fuse-protected circuit | **18 AWG** (19 SWG or 1 mm²) |
| Motor power for TETRIX MAX 12V DC motors and REV Core Hex (REV-14-1300); PWM/servo; LEDs (5V/12V); ≤10A fuse-protected circuit | **22 AWG** (22 SWG or 0.5 mm²) |
| Signal-level circuits (draw ≤1A continuous from a source that can't deliver >1A: I2C, DIO, analog, encoder, RS485) | **28 AWG** (29 SWG or 0.08 mm²) |

- Integrated wires originally attached to legal COTS devices, or wires sold by the manufacturer, count as part of the device and are exempt.
- Box: use wire with clearly labeled sizes. With unlabeled wire, be ready to prove the gauge (e.g. samples). Paralleling smaller wires does NOT count as a larger wire.

### R610 *Use specified wire colors for specific buses.*
12V main power bus and +5V aux bus wires must be color-coded along their entire length:
- A. positive (+12VDC / +5V Aux): red, yellow, white, brown, or black-with-stripe;
- B. negative (common/GND): black or blue.
- Box: doesn't apply to motor wiring, signal wiring (encoder/sensor), servo cables and extensions, or wires originally attached by the manufacturer.
- **Ambiguity:** R601.A says the replacement fuse is installed "per R610," but R610 is about wire colors. This is probably a stale cross-reference, perhaps meant to be R604.

### R611 *Powered USB hubs must draw energy from approved sources.*
Powered USB hubs may be powered only by:
- A. an approved COTS USB battery pack (R602), or
- B. the 5V aux power port on a REV Expansion Hub or Control Hub.

### R612 *Do not modify critical power paths.*
CUSTOM CIRCUITS must not directly alter the power or control pathways between:
- A. the battery and the main power switch;
- B. the main power switch and a power regulating device (per R603);
- C. any two power regulating devices (per R607);
- D. power regulating devices and actuators.
- Box: high-impedance voltage monitoring or low-impedance current monitoring circuits are OK if their effect on power pathways is inconsequential. Altering a pathway includes boost/buck converters or anything else that turns the battery's naturally varying DC into a constant voltage. **Devices that modify actuator control signals or power (beyond those allowed by R505) are prohibited, e.g. the goBILDA Servo Travel Tuner.** **[CODE]** Servo range has to be handled in software (e.g. `scaleRange`) or with the manufacturer's programmer per R504.C, not an inline tuner.
- **Ambiguity:** the cross-references in B ("per R603") and C ("per R607") look off. C presumably means R505-listed devices.

### R613 *Do not mix and match power on or between power regulation devices.* **[CODE]**
For any R505 device:
- A. sensors, encoders, and other devices must be powered only by the regulation device they connect to. No outside sources (custom circuits, a second hub);
- B. a port's power may only feed devices plugged into that exact port. No cross-wiring to other ports, and no combining multiple ports into one bus;
- C. 6V from approved servo power modules/injectors may only power servos;
- D. the +5V Aux port on a REV hub may power devices not connected to other power regulation devices, except via USB.
- Code/hardware impact: the Pinpoint odometry computer must be powered from the I2C port it's plugged into. Don't feed it from a different hub or a separate 5V source.

---

## 12.7 Control, Command & Signals System (p136-140)

### R701 *Control the ROBOT with a single ROBOT CONTROLLER.* **[CODE]**
- Exactly 1 programmable ROBOT CONTROLLER, the only source of actuator control. It is either:
  - A. a REV Control Hub (REV-31-1595), or
  - B. an Android smartphone connected to a REV Expansion Hub (REV-31-1153).
- C. You may add **no more than one** additional REV Expansion Hub (REV-31-1153).
- Max config: Control Hub + 1 Expansion Hub, giving 8 motor ports, 12 servo ports, and 2 IMUs. The 8-motor/8-servo limit of R503 still applies.
- Box: keep control system software up to date (FTC Firmware Update page). **[CODE]** Keep SDK 12.0, hub firmware, and DS app current. The REV Control Hub is the only officially supported RC. Teams using other devices must verify compatibility themselves.

### R702 *Teams may not alter coprocessor software.* **[CODE]**
- Teams may not modify coprocessor software unless this rule allows it. Manufacturer-provided binary firmware updates may be applied as directed.
- Exception: **programmable vision coprocessors natively supported by the FTC SDK** may be reprogrammed. Table 12-9 lists only **Limelight Vision Limelight 3A (LL_3A)**.
- Examples (boxes):
  1. Adafruit BNO055 IMU (onboard Cortex-M0, not user-modifiable): **allowed**.
  2. SparkFun OTOS: **allowed**. Modifying its open-source firmware is NOT allowed, but SparkFun firmware updates are.
  3. Digital Chicken Labs OctoQuad FTC Edition (RP2040 Pico): **allowed**. No custom firmware. Manufacturer binary updates are OK.
  4. Optical flow sensors (vision coprocessor inside): **allowed**, treated like other coprocessors.
  5. DFRobot HuskyLens and Charmed Labs Pixy2 (configurable, not programmable): **allowed**.
  6. **OpenMV Cam, Luxonis OAK-1, and Limelight 3G: PROHIBITED** (programmable vision coprocessors not on the list).
- Code impact: the team's Limelight must be a **3A**, not a 3G. Custom Limelight pipelines (including Python SnapScript) are allowed on the 3A. The goBILDA Pinpoint is a coprocessor too. Use stock firmware or official updates only.

### R703 *Smartphone Android devices used as a ROBOT CONTROLLER must connect to the REV Expansion Hub using USB.*
Through the phone's integrated USB port, using any combination of USB cables (including OTG) and/or USB hubs (powered or not, with or without OTG). This doesn't affect a Control Hub setup.

### R704 *Use networks and bandwidth as directed.* **[CODE]**
- A. No wireless communication to, from, or within the ROBOT other than the provided official tools.
- B. Don't interfere with ROBOT networks. All signals must come only from the RC or DS device on the RC Wi-Fi network. No other device may try to connect to, interfere with, or alter the RC network.
- C. During MATCH play, all programming laptops and other devices (except the DS) must be disconnected from the RC Wi-Fi.
- D. Software with access to the RC Wi-Fi must limit continuous streaming. Robot control data, debugging data, and telemetry may only be streamed **through the FTC Driver Station Application**. **"Additional logging/streaming services, such as those hosted by third party plugins and tools such as FTC Dashboard, FTControl Panels, and others are prohibited. No continuous video stream is allowed."**
- E. If event staff assign a Wi-Fi band or channel, teams must use it.
- **Code impact (HIGH):** this team's subsystems use **Panels (`com.bylazar:fullpanels`)** `TelemetryManager` and `@Configurable`. Panels is named explicitly as prohibited. Competition OpModes must not stream to Panels. Options: route telemetry to the SDK `Telemetry` (DS), or disable or strip the Panels server in competition builds. The Limelight's web stream must also not run continuously over the RC network.
- **Ambiguity:** D doesn't say "during MATCH play" (C does). It may apply any time the robot is on the event network, or only in matches. It's also unclear whether merely having the Panels plugin installed but idle is OK, or whether it must be removed from the APK. Ask Q&A, or disable it in competition builds to be safe.

### R705 *Configure devices for your team number.* **[CODE]**
- A. RC name: `<team number>-RC` (e.g. 12345-RC).
- B. DS name: `<team number>-DS`.
- C. Spares may add a letter: `<team number>-<letter>-RC/DS` (e.g. 12345-A-DS).
- Box: see the DS and RC instructions for renaming.

### R706 *Only specified modifications to core control system devices are permitted.*
The DS device and software, Android RC device, main and secondary power switches, power regulation devices, fuses, and batteries must not be tampered with in any way. That includes drilling, cutting, machining, rewiring, disassembling, painting, and replacing enclosures. Exceptions:
- A. wires/cables/signal lines connected via standard connection points;
- B. fasteners (including adhesives) to attach devices to the OPERATOR CONSOLE or ROBOT, or to secure cables;
- C. thermal interface material;
- D. labeling that doesn't cover product-identifying labels;
- E. moving jumpers/switches per the manufacturer's manual;
- F. manufacturer-supplied firmware updates;
- G. cutting, stripping, and/or connectorizing integral wires on motor controllers and batteries;
- H. repairing devices other than batteries, if performance and specs are identical afterward;
- I. insulating exposed conductors;
- J. tape for debris protection;
- K. modifying or replacing power switch mounting brackets.
- Box: repairs void warranties at the team's risk and must be functionally equivalent. "Repairs" that change connector types, modify the footprint, or add mechanical enhancements are prohibited.

### R707 *USB is for vision.* **[CODE]**
Only these may connect to the control system via USB:
- A. webcams and optical vision sensors per R708;
- B. a USB hub or USB switch;
- C. a REV Expansion Hub.
- So USB-connected IMUs, USB serial microcontrollers, and USB gamepads on the robot side are not allowed.

### R708 *Use only supported USB vision.* **[CODE]**
Only single-image-sensor vision devices natively supported by the RC app. **No stereoscopic cameras.** This includes:
- A. all UVC-compatible USB webcams (Logitech C270, etc.);
- B. vision coprocessors allowed per R702 (Limelight 3A).
- Box: UVC webcams may use only the UVC stream/data. Other interfaces or data from the webcam are off-limits (e.g. a webcam's built-in mic or IMU). To request support for other USB vision devices, email customerservice@firstinspires.org with subject "USB Vision Future Support Request", contact details, and device details.

### R709 *Recording devices are okay.*
Self-contained video recorders (GoPro etc.) are allowed only for non-functional post-MATCH viewing, with wireless turned off.

### R710 *Lasers are only allowed if they're part of a sensor, low-energy, and non-visible.* **[CODE]**
All of these must hold:
- A. part of a sensor;
- B. rated IEC/EN 60825-1 "Class I" or IEC/EN 62471 "Exempt";
- C. non-visible spectrum.
- (Typical ToF distance sensors such as the REV 2m Distance Sensor qualify. Visible laser pointers for aiming do not.)

### R711 *Configure Android devices appropriately.* **[CODE]**
RC and DS Android devices (smartphones, Control Hub, Driver Hub):
- A. Control Hub users must change the Wi-Fi password from the default;
- B. smartphone users must enable Airplane Mode;
- C. Wi-Fi enabled and **Bluetooth disabled** on both the RC and DS;
- D. on the DS, remove all remembered Wi-Fi Direct groups and Wi-Fi connections except the RC's.

---

## 12.8 Pneumatic Systems & Airflow Devices (p141)

### R801 *No Pneumatic Actuators, High-Speed Blowers, or Vacuums.*
- A. Only sealed COTS closed-air systems pre-charged by the manufacturer (e.g. gas shocks) are allowed.
- B. No stored-pressure COMPONENTS actuated by a solenoid or similar device, and none able to change their stable state.
- C. No generating pressure or vacuum.
- D. No user-adjustable gas storage vessels, except air-filled (pneumatic) COTS wheels.
- E. No device that creates high-speed airflow, except cooling fans integrated into COTS computing devices.
- Box: the intent is no pneumatic actuators, pressure/vacuum storage, compressors, vacuum generators, or blowers. Gas springs and dampers are OK. A fan designed to move SCORING ELEMENTS counts as high-speed airflow. **High-speed flywheels or rollers used to manipulate SCORING ELEMENTS are NOT, on their own, high-speed airflow devices.** **[STRATEGY]** The team's dual-flywheel launcher is fine.

---

## 12.9 OPERATOR CONSOLE (p141-142)

### R901 *Use only a specified DRIVER STATION device.* **[CODE]**
- Exactly one approved Android DS device may be connected and powered on at a time. The console must have at least one of:
  - A. REV Driver Hub (REV-31-1596), or
  - B. any Android device, with or without USB cables (including OTG) and/or USB hubs (powered or not, with or without OTG), for connecting one or more gamepads.
- Box: the REV Driver Hub is the only officially supported DS. Spare DS devices are allowed if only one is connected and powered on at a time.

### R902 *The OPERATOR CONSOLE must make the touch screen accessible.*
- The DS touch screen must be accessible, with the display clearly visible during inspection and MATCH. It must work without additional aids (e.g. no mouse required).

### R903 *OPERATOR CONSOLE physical requirements.*
- Max volume, including power sources (power banks): **3 ft wide x 1 ft 6 in deep x 2 ft tall (91.4 x 45.7 x 61.0 cm)**. Items held or worn by DRIVERS during the MATCH are excluded.
- Box: see R203 (fair play/ARENA safety). There's no hard weight limit, but consoles **over 20 lb (~9 kg)** get extra scrutiny. A spare external USB hub is OK if only one is connected at a time. The intent is a container for the DS and support electronics, not a robot cart or a replacement for the provided stand or table.

### R904 *ROBOT application wireless communication only.* **[CODE]**
- Other than the RC app / DS app connection, no wireless communication to, from, or within the OPERATOR CONSOLE during a MATCH.
- Box: prohibited examples include active wireless network cards, **wireless gamepad communication**, and Bluetooth devices. Gamepads must be wired (USB) to the DS. Bluetooth/wireless PS4/PS5 or Xbox pads are not allowed.

---

## 13 Tournament (T) (p143-162)

### 13.1 Overview
- Three MATCH types: Practice (not at all events), Qualification (earn MATCH points and RANKING POINTS, which set seeding), and Playoff (decide the winning ALLIANCE).
- These rules apply to all Section 4 event types. Section 14 adds rules for League Meets and League Tournaments.

### 13.2 MATCH Replays

**T201 *Replays are allowed, but rare.***
- Replays happen only for an ARENA FAULT, or when FIELD STAFF stopped the MATCH anticipating FIELD damage or injury.
- ARENA FAULT examples:
  - A. FIELD elements broken by normal gameplay;
  - B. FIELD elements broken by ROBOT abuse that affects the outcome **for their opponents**. Box: breakage that affects the abuser's own ALLIANCE outcome is NOT an ARENA FAULT;
  - C. FIELD elements moving beyond tolerance, not due to robots;
  - D. widespread wireless interference affecting multiple ROBOTS, typically at once and on both ALLIANCES;
  - E. MATCH timer display failure;
  - F. FIELD STAFF errors (except those in 10.8 Other Logistics).
- If the Head REFEREE rules an ARENA FAULT affected the outcome and any team on the affected ALLIANCE wants a replay, it's replayed. FIRST HQ may also replay any MATCH where an ARENA FAULT impacts the event outcome.
- Box: team-fault behaviors (low battery, **programming issues**, mechanical problems) are NOT grounds for a replay. **[CODE][STRATEGY]** Software crashes or disconnects caused by your code get no replay. "Outcome affected" means it changes which ALLIANCE won and/or the RP assignment. "Event outcome affected" means, per FIRST HQ, it changes the RP assignment or dramatically affects ranking-criteria points. ARENA FAULTs that don't affect the outcome get no replay, e.g. plastic falling far from activity, a delayed ARENA sound, or adjusted/delayed penalty or score assignment (including post-match).

**T202 *Replays will replicate the conditions of the original MATCH.***
- A. A ROBOT absent or DISABLED before the original MATCH started stays DISABLED for the replay.
- B. The same FIELD is used unless damage requires otherwise.
- C. Exception: starting locations and pre-loaded SCORING ELEMENTS don't need to match the original. **[STRATEGY]** You can change your start position or preload in a replay.
- Box: environmental factors such as ambient lighting may differ. **[CODE]** Vision should tolerate lighting changes.

### 13.3 Clarifications on MATCH Play Results ("Question Box")
- Up to 2 reps wearing DRIVE TEAM badges may go to the Question Box. The Head REF or FTA may postpone until after the next MATCH.
- The FTA handles technical FIELD/ROBOT questions, and other team members may join if needed. Rulings and results questions go to the Head REFEREE per T301, after results are shown.
- REFEREES are told **not** to self-track MINOR/MAJOR FOUL details. Don't expect them to recall who, when, or what.
- Box: any reasonable question is fair, and Head REFs try to give helpful feedback (how and why fouls are called, why a design may be susceptible), but they may not have specifics.

**T301 *Head REFEREE interactions.*** At most 2 people, at least 1 of them a STUDENT.
- Violation: the Head REFEREE won't address extra or non-compliant members or side conversations.
- Box: some events restrict ARENA access to the DRIVE TEAM. Teams may swap buttons internally to reach the Question Box. The STUDENT should actively participate. Don't record without consent (E116).

**T302 *MATCH questions must be timely.***
- A. Qualification MATCH questions: any time before ALLIANCE selection begins, or within 5 min of the last Qual MATCH at events without Playoffs.
- B. Playoff MATCH questions: before the next Playoff round starts. For the final Playoff MATCH, immediately after it.
- Box: it's best to ask within 3 MATCHES. Questions more than 5 min after the last Playoff MATCH likely won't be addressed.

**T303 *Keep questions factual and constructive.*** Prepare in advance and bring rule or Q&A references.
- Box: there are no negative implications for self-advocacy. Remember Core Values. Results may be on FTC-Events.

### 13.4 General Tournament Rules

**T401 *The Head REFEREE has ultimate and final authority regarding gameplay.***
- They may take input from FIRST staff, FTA, ED, PDP, and others. Rulings are final. **No event staff will review video, photos, or renderings of any MATCH from any source under any circumstances.**
- A. A RED or YELLOW CARD must have the violated rule recorded.
- B. EDs and PDPs may not overrule the Head REF.
- C. Framework of Behaviors and Competition Integrity Contract (CIC) violations may escalate beyond the Head REF's ruling.
- D. (Text-extraction split.) Every Qual and Playoff MATCH must be observed by a certified Head REF, who watches only 1 MATCH at a time.
- Box: rules are written for human enforcement, and some depend on judgment. The "right call" is the one made in good faith with the information available.

**T402 *Only REFEREES can declare a ROBOT DISABLED.*** A robot is DISABLED only once a REFEREE declares it during a MATCH, for a violation or a ROBOT failure. For a violation, the REF may first have the team drive the robot to a neutral position.

**T403 *The Event Director has ultimate authority over all non-gameplay decisions*,** e.g. A. venue access per the public schedule; B. pit sizes and utilities; C. health and safety; D. registration and eligibility; E. team conduct away from the ARENA.

**T404 *All competition FIELDS at an event must be consistent with each other*,** e.g. A. elevation off the floor; B. FIELD display monitors; C. perimeter type; D. TILE size and type.
- Box: practice fields need not match each other or the competition fields. **[CODE]** Don't over-tune to a practice field.

**T405 *During optional FIELD measurement and calibration time(s), ROBOTS may not practice on the FIELD.*** **[CODE][STRATEGY]**
- When the ARENA is open for measurement, robots may run OpModes but the CHASSIS may not move under its own power.
- **Violation: VERBAL WARNING. YELLOW CARD for subsequent violations during the event.**
- Box: at ED discretion, the ARENA may be open at least 30 min before Quals for surveying, measuring, and sensor calibration.
  - Allowed: A. power on; B. init an OpMode; C. operate/extend MECHANISMS outside the CHASSIS; D. CONTROL SCORING ELEMENTS; E. connect programming laptops and other devices; F. team members on the FIELD; G. manually move the robot to positions (not under its own power); H. measure the FIELD with tools or sensors.
  - Not allowed: I. CHASSIS driving under its own power (no AUTO/TELEOP driving); J. **LAUNCHING SCORING ELEMENTS**; K. team-member practice (e.g. repeated HP placement).
- **Code tip:** use this window to verify Limelight AprilTag relocalization, Pinpoint pose, and turret aim against real HIVE positions by pushing the robot by hand and running a non-driving, non-launching diagnostic OpMode (turret tracking and flywheel spin-up are OK, but don't feed balls). Laptops may connect (Panels is usable here, since E allows connected devices, though R704 is still ambiguous).

**T406 *No team timeouts, but MATCHES have breaks.***
- A. Qualification: at least **5 min** from posting of the previous MATCH's results to the expected start of the next (for G301).
- B. Playoff: at least **8 min**.
- Box: if results won't be posted (e.g. an immediate replay), the Head REF allows reasonable reset time. Breaks are tracked by the event software, and FIELD STAFF communicate start times. **[STRATEGY]** Battery swap and repair windows: 5 min in Quals, 8 min in Playoffs.

**T407 *MATCHES are played in order.*** Numerical order unless extenuating circumstances (Head REF with ED). All Quals before ALLIANCE selection, and all of a Playoff round before the next round. Teams are told the timing of out-of-order or replayed MATCHES.
- Box: examples include A. a replay at the next break, end of day, end of Quals, or end of the current playoff round; B. a lengthy repair on one FIELD while others continue; C. urgent team circumstances. T406 and G301 still apply.

### 13.5 Practice MATCHES (p149)
- Played before Quals where offered. The schedule is out no later than the start of Practice (possibly on FTC-Events). Assignments are random and can't be swapped. Each team gets an equal number, unless teams x matches isn't divisible by 4, in which case some random teams get an extra. Not guaranteed at all events.
- **13.5.1 Filler Line:** fills empty Practice slots first come, first served (or all slots at open-schedule events). Size depends on venue. Criteria: A. passed inspection (may be waived at open-schedule events); B. DRIVE TEAM joins with the ROBOT; C. no robot work while in line; D. at most 1 spot per team; E. can't join while queued for your own Practice MATCH. **[STRATEGY]** It's extra field time for testing auto.

### 13.6 Qualification MATCHES (p149-151)
- **13.6.1 Schedule:** published at least 15 min before Quals begin. Only eligible, checked-in teams are included (I102). Delivered as a printed copy, a posted hard copy, or a digital display, and possibly on FTC-Events. Each round is 1 MATCH per team. **5 or 6 Qual MATCHES per team** (ED decides). Championship, Premier, and Regional Championship events may schedule more. Figure 13-1 is a sample schedule; SURROGATE matches are marked. The scheduling algorithm reference is the Idle Loop software website.
- **13.6.2 MATCH Assignment:** the software assigns 1 partner per Qual MATCH, with no swaps. Priority order: (1) minimum time between matches (varies by event size); (2) minimize repeat partners; (3) minimize repeat opponents; (4) minimize SURROGATE use; (5) even red/blue distribution. All teams play the same number of matches, unless teams x matches isn't divisible by 4. Then random teams play an extra as a **SURROGATE**: marked with * on the schedule, **always their 3rd Qual MATCH**, and the result doesn't affect their ranking. **YELLOW/RED CARDS earned as a SURROGATE still carry forward.** Back-to-back matches get the T406 minimum break.
- **13.6.3 Qualification Ranking:** RP is credited per Table 10-2 after each Qual MATCH. RANKING SCORE (RS) is the average RP over Qual MATCHES, excluding the SURROGATE match. Teams are ranked 1..n by RS. SURROGATE matches are excluded from all calculations. **A DISQUALIFIED match contributes 0 to all sort criteria.**
- **Table 13-1 Ranking tiebreakers** **[STRATEGY]**:
  1. RANKING SCORE (RS)
  2. Average ALLIANCE MATCH points, excluding MINOR and MAJOR FOULS (match points minus fouls)
  3. **Average number of TIPS**
  4. Average AUTO points
  5. Random sort
  - TIPs (flipping the HIVE) are the 3rd tiebreaker, and AUTO points are the 4th, so auto scoring matters for seeding.

**T601 *DISQUALIFICATION applies only to the DISQUALIFIED team in Qualifications.*** A team's DQ in Quals doesn't affect its partner.

### 13.7 Playoff MATCHES (p152-159)
- Teams play on fixed ALLIANCES from selection, in a **double-elimination** bracket. No RP; advancement is by winning. **In Playoffs, a DQ applies to the whole ALLIANCE (all get 0 MATCH points).** **[STRATEGY]**

**T701 *Send one or two STUDENT representatives*** to ALLIANCE selection (typically right after the last Qual MATCH).
- Violation: teams that don't send one are **ineligible for playoffs**.
- Box: if an absent team would have been an ALLIANCE lead, lower leads move up 1. Teams not planning to play should tell the ED and Head REF early.

**T702 *Declining teams cannot be picked.*** A captain may not invite a team that declined another ALLIANCE's invitation.
- Violation: the captain must pick again.
- Box: a lead who declines can still pick for their own ALLIANCE but can't be invited by another.

**T703 *There are no backup teams in Playoff MATCHES.***
- Box: consider reliability when picking, since all ALLIANCE teams must play every round. **[STRATEGY]** Reliability matters; with 2-team alliances, both robots play every match.

**T704 *During Playoff MATCHES, teams may have more ARENA access.*** At ED direction, up to **3 additional pit crew** per team, with the same ARENA access as the DRIVE TEAM, for repairs only (no MATCH play). This is venue-specific and at ED discretion.

**T705 *Multiple DISQUALIFICATION is handled specially* (Playoffs):** A. a DQ'd ALLIANCE loses; B. if both are DQ'd, the one DQ'd first chronologically loses; C. if both are DQ'd simultaneously (per the Head REF), the MATCH is a tie.

**13.7.1 ALLIANCE Selection Process**
- The top-ranked teams become ALLIANCE leads. The lead's designated STUDENT rep is the ALLIANCE CAPTAIN, who may change between selection and Playoffs. ALLIANCES are numbered 1..N (Table 13-2). Each lead picks 1 team. If one lead accepts another lead's invitation, lower leads move up and the highest-ranked unselected team becomes the newest lead.
- If complete ALLIANCES can't be formed (declines, early departures), play continues with incomplete ones. A 0-team ALLIANCE means an automatic win for the opponent (MATCH skipped). A 1-team ALLIANCE plays 1 vs 2.

**13.7.2 Playoff MATCH Bracket**
- Bracket size is based on playoff-eligible teams, meaning all teams that took part in Quals. Box: no-shows and awards-only teams are excluded. Teams that played Quals but won't play Playoffs are still included.
- **Table 13-2:** 4-10 teams: 2 ALLIANCES; 11-20: 4; 21-40: 6; 41-64: 8.
- Double elimination with upper and lower brackets. With 2 ALLIANCES, they meet directly in the finals. Losing in the upper bracket drops you to the lower bracket. 2 total losses eliminate you.
- **Ties replay until there's 1 winner.**
- Round 1: the higher-ranked ALLIANCE is **red**. Later rounds follow Figure 13-2 regardless of rank. **[STRATEGY/CODE]** Auto must handle both alliance colors. `CrossOpModeStorage` alliance color must be set correctly every match.
- Up to 6 rounds, with breaks between later rounds. The expected start is the scheduled time or 8 min after either ALLIANCE's previous MATCH ends, whichever is later. Replays and tie-breaker matches get a minimum 8-min delay unless all teams are ready sooner, and must be played before the next round.

**13.7.3 2-ALLIANCE bracket timing (Table 13-3):** a 15-min break (Judges' Choice*, Innovate/Design/Control) at t=0; Finals 1 (A2 blue vs A1 red) at 15; 15-min break (Sustain/Reach/Connect) at 18; Finals 2 at 33 (gap 0:15); 15-min break (Think) at 36; Finals 3* at 51 (gap 0:10); awards (Compass*, Finalists, Winners, Inspire) at 54. (*if required; awards may be given after the bracket at ED discretion.)

**13.7.4 4-ALLIANCE timing (Table 13-4):**
- R1: M1 Upper A4(B) v A1(R) at 0 (W→M4 R, L→M3 R); M2 Upper A3 v A2 at 6 (W→M4 B, L→M3 B); 15-min break at 9.
- R2: M3 Lower L2 v L1 at 25 (gaps 0:08/0:14; W→M5 B; L = 4th); M4 Upper W2 v W1 at 31 (0:14/0:20; W→M6 R; L→M5 R).
- 15-min break (Judges' Choice, Design, Reach) at 33.
- R3: M5 Lower W3 v L4 at 48 (W→M6 B; L = 3rd).
- 15-min break (Control, Innovate, Sustain) at 51.
- Finals M6 W5 v W4 at 66 (0:15/0:33); 15-min break (Connect, Think) at 69; Finals M7* at 84; awards (Compass*, Finalists, Winners, Inspire 2,1) at 87.

**13.7.5 6-ALLIANCE timing (Table 13-5):** A1 and A2 get Round 1 byes.
- R1: M1 A5 v A4 (0); M2 A6 v A3 (6).
- R2: M3 W1 v A1 (12); M4 W2 v A2 (18).
- R3: M5 Lower L2 v L3 (24; loser tied 5th); M6 Lower L1 v L4 (30).
- R4: M7 Upper W4 v W3 (36); M8 Lower W5 v W6 (42; loser 4th).
- Break (Judges' Choice, Design 2,1, Reach 2,1) at 45.
- R5: M9 Lower W8 v L7 (60; loser 3rd).
- Break (Control, Innovate, Sustain) at 63.
- Finals M10 W9 v W7 (78); break (Connect, Think) at 81; Finals M11* (96); awards (Compass*, Finalists, Winners, Inspire 3,2,1) at 99.
- (Text extraction garbles some rows. The figure has the authoritative bracket.)

**13.7.6 8-ALLIANCE timing (Figure 13-6, Table 13-6):**
- R1: M1 A8 v A1 (0); M2 A5 v A4 (6); M3 A7 v A2 (12); M4 A6 v A3 (18).
- R2: M5 Lower L2 v L1 (24; loser tied 7th); M6 Lower L4 v L3 (30); M7 Upper W2 v W1 (36); M8 Upper W4 v W3 (42).
- R3: M9 Lower W6 v L7 (48; loser tied 5th); M10 Lower W5 v L8 (54).
- R4: M11 Upper W8 v W7 (60); M12 Lower W9 v W10 (66; loser 4th).
- Break (Judges' Choice, Design 3,2,1, Reach 3,2,1) at 69.
- R5: M13 Lower W12 v L11 (84; loser 3rd).
- Break (Control, Innovate, Sustain) at 87.
- Finals M14 W13 v W11 (102); break (Connect, Think) at 105; Finals M15* (120); awards at 123.

### 13.8 Dual Division Events (p159-162)
- Larger events run as 2 parallel competitions (each with at least 2 FIELDS) and concurrent judging. The two division-winning ALLIANCES then play for the event title.
- Division assignment methods (PDP's choice): A. Random (shuffle and split; a random division gets the extra team); B. FIM method (brute-force randomizer balancing a performance metric from each team's two best events, by quartiles); C. Alternating by team number (lowest to Div 1); D. Child League Split-up (League Tournaments only); E. OPR method (highest season OPR, alternating assignment); F. Manual upload by the PDP.
- **T801 *Divisions will use the same size playoff brackets*,** based on the playoff-eligible teams in the **smaller** division (Table 13-2).
- **T802 *Dual Division awards are given at the event level*,** not per division.
- **T803 *Dual Division award counts are based on total teams*** (Table 6-1).
- **13.8.1 Advancement points (Table 13-7)** replace the playoff advancement points of Table 4-1: 40 for 1st (Winners); 20 for 2nd (Finalists); 10 for all Division Finalists; 5 for all Division 3rd places.
- **13.8.2 Dual Division Playoffs:** standard Quals and divisional playoffs. The two division winners reset their records and play a 2-ALLIANCE double-elimination grand final (Figure 13-7). First to 2 wins takes the event. Divisions are ranked by Table 13-1 criteria, excluding RANKING SCORE.
- Table 13-8 (6-ALLIANCE dual-division timing) follows the same divisional bracket as Table 13-5, then: end of divisional playoffs plus awards at 99; Event Finals 1 (DivA v DivB) at 114; Event Finals 2 at 132; Event Finals 3* at 150; awards at 153.

---

## 14 League Play Tournaments (L) (p163)
- Not all regions run leagues. A league is a closed group of teams playing multiple League Meets over weeks or months, with roughly equal match counts. **At least 10 League Meet MATCHES** per team.
- League Meets: 5-6 Qual MATCHES per team, no Playoffs or judging.
- **VERBAL WARNINGS and CARDS clear at the end of each League Meet** (in addition to 10.6.1 behavior).
- 1 League per team per season, and only 1 League Tournament. A team may join a league outside its region if it's their only league. A team can't advance from a League Tournament to an out-of-region Regional Championship unless both regions' PDPs agree to move the team for the whole season.
- League Tournament rankings (for ALLIANCE selection and Qualification Round Performance Points) combine each team's **top 10 League Meet MATCHES** (sorted by Table 13-1) with its League Tournament MATCHES. Teams with fewer than 10 meet matches get 0 for the missing ones. **[STRATEGY]** Attend enough meets to fill all 10 slots.
- League Tournament advancement (Table 4-1) uses only League Tournament performance, except Qualification Round Performance, which uses the combined ranking above.

## 15 FIRST Championship (C) (p165-166)
- 2026-27 Championship: **6 divisions**, each running standard Quals and Playoffs. The 6 division-winning ALLIANCES go to the Championship Playoffs on the Championship FIELD(s) (15.5).
- **15.1 Awards:** judging may be modified, with details by the last regularly scheduled Team Update. Awards are per division except (Table 15-1): Inspire is 1st/2nd/3rd per division and 1st Place at the Championship level; FIRST Leadership List: 0 per division, 10 Championship; Compass: 0 per division, 1 Championship.
- **15.2 Game Modification:** the number, type, and distribution of SCORING ELEMENTS, and the RP thresholds, may change for Championship, published by the last regular Team Update. **Division FIELDS sit on risers about 24 in (60.95 cm) high, with drivers and staff at floor level.** Some practice fields are elevated too. **[STRATEGY/CODE]** Driver sightlines change, and vision and lighting may differ. Perimeters are anchored, without AndyMark Perimeter Straps. Aesthetic changes (decals, metal coatings, materials, lights) are possible. They're intended to be purely aesthetic, but **[CODE]** they could affect vision or color sensing. Watch for the Team Update.
- **15.3 3-ROBOT ALLIANCES:** selection round 1 per 13.7.1, then round 2 in reverse order (A8 picks first, A1 last), giving 8 ALLIANCES of 3. Any 2 of 3 robots start each Division or Championship Playoff MATCH. The choice needn't be declared in advance but must not delay the match (G301). Box: changing the pair after 2 robots have left queuing is likely a delay of MATCH.
  - **C301 *Replays use the same ROBOTS.*** A replayed Playoff MATCH must use the same 2 robots, unless the Head REF judges the ARENA FAULT rendered a robot inoperable. A tie-breaker match may use any 2 of 3. Box: Championship has many more matches, and the 3rd robot gives backup and strategic flexibility.
- **15.4 Pit Crews:** each team on a Championship Playoff ALLIANCE may have 3 additional pit crew (adults or STUDENTS) inside the ARENA, per T704, staying in the ARENA pit areas.
- **15.5 Championship Playoffs:** structure TBD in a future Team Update.

---

## 16 Glossary (p167-171), verbatim substance
Intro: defined terms are in ALL CAPS. Rules mean what they plainly say. Undefined words take their common conversational meaning.

| Term | Definition |
|---|---|
| ALLIANCE | a cooperative of 2 FIRST Tech Challenge teams |
| ALLIANCE AREA | an approximately 97 in. (246.40 cm) wide by 54 in. (137.15 cm) deep by infinitely tall volume formed by placing ALLIANCE colored tape onto the flooring surface outside of the FIELD |
| ALLIANCE CAPTAIN | the designated STUDENT representative from each ALLIANCE lead |
| ARENA | includes all elements of the game infrastructure that are required to play this season's FTC game including: the FIELD, SCORING ELEMENTS, queue area, team media area, and all equipment needed for FIELD control, ROBOT control, and scorekeeping |
| ARENA FAULT | an error in ARENA operation |
| AUTO | the first 30 seconds of the MATCH, during which DRIVERS may not provide input to their ROBOTS, so ROBOTS operate with only their pre-programmed instructions **[CODE]** |
| CELL | a three-dimensional structure that can hold NECTAR and POLLEN |
| CHASSIS | ROBOT'S MAJOR MECHANISM that enables it to move around a FIELD |
| COMPONENT | any part in its most basic configuration, which cannot be disassembled without damaging or destroying the part or altering its fundamental function |
| CONTINUOUS | describes durations that are more than approximately 10 seconds |
| CONTROL | an action by a ROBOT in which the SCORING ELEMENT is fully supported by or stuck in, on, or under the ROBOT or it intentionally pushes a SCORING ELEMENT to a desired location or in a preferred direction (i.e., herding). CONTROL requires contact with a ROBOT, either directly or transitively through other SCORING ELEMENTS. Typically, CONTROL requires one of: A. the SCORING ELEMENT is fully supported by the ROBOT; B. the ROBOT is moving the SCORING ELEMENT in a preferred direction with a flat or concave face of the ROBOT **[STRATEGY]** (relevant to the 4-element limit) |
| COTS | a standard (i.e., not custom order) part commonly available from a VENDOR for all teams for purchase |
| CUSTOM CIRCUIT | any active electrical item that is not an actuator (specified in R501) or power regulation device (specified in R505) |
| DISABLED | the REFEREE instructs the team to stop the ROBOT, which will deactivate all outputs, rendering the ROBOT inoperable for the remainder of the MATCH |
| DISQUALIFIED | the state of a team in which they receive 0 MATCH points and 0 RANKING POINTS in a Qualification MATCH or causes their ALLIANCE to receive 0 MATCH points in a Playoff MATCH |
| DRIVE COACH | a guide or advisor |
| DRIVE TEAM | a set of up to 4 people from the same FIRST Tech Challenge team responsible for team performance for a specific MATCH |
| DRIVER | an operator and controller of the ROBOT |
| DRIVER STATION | Android device (smartphone or REV Driver Hub) that runs the DRIVER STATION App software to communicate with a ROBOT per R901 |
| FABRICATED ITEM | any COMPONENT or MECHANISM that has been altered, built, cast, constructed, concocted, created, cut, heat treated, machined, manufactured, modified, painted, produced, surface coated, or conjured partially or completely into the final form in which it will be used on the ROBOT |
| FIELD | an approximately 144 in. by 144 in. (365.75 cm by 365.75 cm) area bounded by the inside surface of the walls **[CODE]** (matches the 144x144 in coordinate frame) |
| FIELD STAFF | volunteers present in and around the ARENA that are responsible for making sure the MATCHES are cycled through efficiently, fairly, safely, and with a spirit of cooperation, Gracious Professionalism®, and generosity of spirit |
| FLOWER | a structure on the FIELD in which POLLEN and NECTAR can be placed into the top, and POLLEN can be removed from the bottom |
| FTA | FIRST Technical Advisor |
| GARDEN | an approximately 23 in. (58.40 cm) by 2 in. (5.10 cm) wide and infinitely tall volume defined by the outside edge of blue or red tape in opposite corners of the FIELD (**ambiguity:** "23 in. by 2 in. wide" looks like a typo or extraction issue; the dimensions seem odd for a corner zone. Check the field drawings.) |
| HIVE | a bi-stable structure made up of two CELLS and a connecting assembly that rotates on a pivot |
| HOME REGION | the region in which a team is assigned and in which they are only eligible to advance from events within |
| HUMAN PLAYER | a SCORING ELEMENT manager |
| INSPECTOR | a person determined by FIRST to accurately and efficiently assess the legality of a given part or the whole of a ROBOT, an event volunteer role |
| JUDGE | volunteers who determine the teams that receive awards at events |
| LAUNCH/LAUNCHING | an action by a ROBOT in which the SCORING ELEMENT is shot into the air, propelled across the floor to a desired location or in a preferred direction, or thrown in a forceful way |
| LEAVE | a scoring achievement in which a ROBOT must move so that it is no longer contacting the perimeter wall **[CODE]** (auto must drive off the wall) |
| LOADING ZONE | an approximately 23 in. (58.40 cm) wide by 11 in. (27.95 cm) deep infinitely tall volume bounded by red or blue tape and the adjoining FIELD perimeters |
| LRI | Lead ROBOT INSPECTOR |
| MAJOR FOUL | a credit of 20 points towards the opponent's MATCH point total |
| MAJOR MECHANISM | a group of COMPONENTS and/or MECHANISMS assembled together to address at least 1 game challenge: ROBOT movement, SCORING ELEMENT manipulation, FIELD element manipulation, or performance of a scorable task without the assistance of another ROBOT |
| MATCH | a 30-second AUTO period, an 8-second transition period between AUTO and TELEOP, and a 2-minute TELEOP period in which the ROBOT plays the current season game **[CODE]** (30 s auto timer; 8 s gap for OpMode switch) |
| MECHANISM | an assembly of COMPONENTS that provide specific functionality on the ROBOT. A MECHANISM can be disassembled (and then reassembled) into individual COMPONENTS without damage to the parts |
| MINOR FOUL | a credit of 5 points towards the opponent's MATCH point total |
| MOMENTARY | describes durations that are fewer than approximately 3 seconds |
| NECTAR | approximately 3.6 in. (9.1 cm) Gopher ResisDent™ polyethylene balls (no color given here) |
| OPERATOR CONSOLE | the set of COMPONENTS and MECHANISMS used by the DRIVE TEAM to relay commands to the ROBOT |
| PARK | a scoring achievement in which a ROBOT must move so that it is at least partially in the LOADING ZONE **[CODE]** (end-of-match/auto target) |
| PIN/PINNING | an action by a ROBOT that is preventing the movement of an opponent ROBOT by contact, either direct or transitive (such as against a FIELD element) |
| POLLEN | 2.8 in. (7.1 cm) Gopher ResisDent™ polyethylene balls in yellow **[CODE]** (color/size for detection) |
| POLLINATOR 1 RP | an RP earned when the number of TIPS at or above threshold |
| POLLINATOR 2 RP | an RP earned when the number of TIPS at or above threshold (**ambiguity:** both entries are identical; thresholds are in Table 10-2, and presumably POLLINATOR 2 has the higher one) |
| PORTFOLIO | a document used as part of the judging process with requirements outlined in A201 |
| RANKING POINTS (RP) | credited to a team based on their ALLIANCE'S performance in Qualification MATCHES |
| RANKING SCORE (RS) | the average number of RANKING POINTS earned by a team throughout their Qualification MATCHES |
| RED CARD | a penalty issued by the Head REFEREE for egregious ROBOT or team member behavior or rule violations which results in a team being DISQUALIFIED for the MATCH |
| REFEREE | an official who is certified by FIRST to enforce the rules of the current season's game, an event volunteer role |
| REPEATED | describes actions that happen more than once within a MATCH |
| ROBOT | an electromechanical assembly built by a FIRST Tech Challenge team to play the current season's game and includes all the basic systems required to be an active participant in the game: power, communications, control, and movement about the FIELD |
| ROBOT CONTROLLER | Android device (a REV Control Hub (REV-31-1595) or a smartphone Android device connected to a REV Expansion Hub (REV-31-1153)) that runs the ROBOT CONTROLLER app to control the ROBOT as defined in R701 |
| ROBOT SIGN | a required assembly which attaches to the ROBOT and simultaneously identifies a ROBOT'S team number as well as its ALLIANCE affiliation |
| SCORING ELEMENT | SCORING ELEMENTS for BIOBUZZ are POLLEN and NECTAR |
| STRATEGIC | describes actions done with the aim of gaining a competitive advantage |
| STARTING CONFIGURATION | the physical configuration in which a ROBOT starts a MATCH |
| STUDENT | a person who has not completed high-school, secondary school, or the comparable level in their HOME REGION as of September 1st |
| SURROGATE | a team randomly assigned by event management software to play an extra Qualification MATCH |
| SWARM RP | an RP earned when the combined LEAVE + PARK points earned at or above threshold **[STRATEGY]** (LEAVE in auto and PARK both feed an RP) |
| TELEOP | third period of each MATCH is 2 minutes (2:00) long and called the teleoperated period (TELEOP). During TELEOP, DRIVERS remotely operate ROBOTS |
| TILE | flooring surface of the FIELD is made of 36 interlocking soft foam TILES (a 6x6 grid of 24 in tiles) |
| TIP/TIPPED | a scoring criteria in which the HIVE moves from one stable state to the other stable state with the downwards-facing CELL becoming the upwards-facing CELL and the damper on the HIVE that was previously not contacting the frame begins to contact the frame **[STRATEGY/CODE]** (after a TIP, which CELL is up/down changes, which affects LaunchTargets) |
| VENDOR | a legitimate business source for COTS items that satisfies criteria defined in Section 12 ROBOT Construction Rules (R) |
| VERBAL WARNING | a warning issued by event staff or the Head REFEREE |
| YELLOW CARD | a warning issued by the Head REFEREE for egregious ROBOT or team member behavior or rule violations |

---

## Summary of code/config checklist from this range
1. **R704.D: Panels and FTC Dashboard streaming are prohibited.** The current subsystems depend on Panels `TelemetryManager`. Competition builds need to fall back to the DS `Telemetry` and keep the Panels server off. No continuous video stream (Limelight web UI/stream).
2. **R702/R708: Limelight 3A is the only allowed programmable vision coprocessor.** Limelight 3G, OAK, and OpenMV are banned. No stereo cameras. Other coprocessors (Pinpoint, OTOS, OctoQuad) take stock or manufacturer firmware only.
3. **R503: at most 8 motors and 8 servos**, summed across all configurations. **R701: Control Hub + at most 1 Expansion Hub.** R505: at most 2 motors or 2 servos per port.
4. **R707: USB only for UVC webcams / Limelight 3A, USB hubs/switches, and an Expansion Hub.**
5. **R711/R904: Bluetooth off, wired gamepads only**, non-default Control Hub Wi-Fi password, DS keeps only the RC network. **R705:** name devices `<team#>-RC` / `<team#>-DS`.
6. **R612:** no inline servo signal modifiers (e.g. goBILDA Servo Travel Tuner). Limit servo range in software or with the manufacturer's programmer.
7. **R613/R602:** sensors (Pinpoint, etc.) must be powered from the port they're connected to and from the main battery.
8. **T405:** during field calibration time you may run OpModes, move mechanisms, and spin up, but **no driving under power and no launching** (verbal warning, then YELLOW CARD).
9. **T201:** programming or low-battery failures never get replays. **Table 13-1:** TIPs are the 3rd tiebreaker and AUTO points the 4th.

---

