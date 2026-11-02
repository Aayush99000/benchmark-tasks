# Robocurve Benchmarking Program: tasks

Every team submits its tasks to this repository as pull requests. Program details, the review checklist
and the simulation walkthrough are at https://tasks.robocurve.org.

## Submitting a task

1. **Upload your videos and screenshot** on the task board's [Submit a task](https://tasks.robocurve.org/submit)
   page (or with the command below). You get a media ID, such as `M-7K2Q9F`.

       curl -X POST https://tasks.robocurve.org/api/media \
         -H "Authorization: Bearer $TEAM_CODE:$TEAM_TOKEN" \
         -F video=@model_run.mp4 -F demo=@demo.mp4 -F screenshot=@setup.png

   Upload videos as H.264 MP4 (`ffmpeg -i in.mp4 -c:v libx264 -pix_fmt yuv420p -movflags +faststart out.mp4`).
   The demo is required when the Dream-1 run doesn't reach full score.

2. **Fork this repository and add one folder** for the task:

       tasks/<task-name>/
         task.json     the task's details, including your media ID
         README.md     the commands to run and score a trial, the runtime per trial, and any seeds
         ...           your scene and scoring code

3. **Open a pull request** into `main`:

       gh repo fork robocurve/benchmark-tasks --clone && cd benchmark-tasks
       git checkout -b lamp-task
       git add tasks/lamp-task && git commit -m "Add lamp task"
       git push -u origin lamp-task
       gh pr create --repo robocurve/benchmark-tasks --fill

Within a minute or two, a comment on your pull request says either that the task was received (with its
ID) and is waiting for review, or what to fix. Push to the same branch to try again.

## task.json

    {
      "title": "Extend a telescoping lamp",
      "category": "Societal relevance",
      "keywords": ["bimanual", "telescoping"],
      "instruction": "Extend the lamp to its full height.",
      "context": "Optional: 1-2 lines on why the task matters, beyond the instruction.",
      "setup": "Every object with type, size and material, in one fixed starting layout matching the screenshot.",
      "scoring": "0.25 at 2 cm of extension; 0.5 at 4 cm; 1.0 at 6 cm or more with the base flat.",
      "run_limit": "2,400 steps (2 minutes at 20 Hz)",
      "reset_notes": "Optional: anything consumed or hard to restore between trials.",
      "fidelity": "How faithful or approximate the simulation is, and what's simplified.",
      "scores": "Dream-1 run: 0.0; demo: 1.0",
      "media": "M-7K2Q9F"
    }

- **Required:** title, category, instruction, setup, scoring, run_limit, fidelity, scores and media.
- **category:** `Impact on jobs`, `Impact on AI timelines`, `Safety and alignment`, `Societal relevance` or `Other`.
- **run_limit:** the most actions a model gets: control steps for action policies like Dream-1 (20 a second),
  or model calls for language-model agents. Choose what the task reasonably needs.
- **scores:** the score each video reached, as your scorer reported it.

## Rules

- **One task per pull request,** one folder under `tasks/`. Up to 5 submissions in any 24 hours.
- **Accepted:** we merge your pull request. **Not accepted:** we close it with feedback; to revise, push your
  changes and reopen it at least 24 hours after the review.
- **The first team to submit a task owns it** while it's pending, and through one revision if it isn't
  accepted. Near-duplicates from other teams during that time aren't accepted.
- **Don't merge pull requests yourselves** (`main` is protected). A task waiting for review doesn't change: to change it, close the
  pull request (which withdraws it) and open a new one.
- **Review is blind:** the task's text, screenshot and videos must not identify your team. Your code may.
- Pull requests are matched to teams by their author, so open them from a GitHub account your team gave us.
