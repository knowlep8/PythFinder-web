/**
 * The clock behind play, pause and the scrubber.
 *
 * It knows nothing about robots: it walks a time from 0 to the length of the
 * run, in real time, and says where it has got to. What that time looks like
 * on the mat is main.ts's problem.
 */

export interface PlaybackHandlers {
  /** where the run has got to, and whether it is still moving */
  onTick: (ms: number, playing: boolean) => void;
}

export class Playback {
  private handlers: PlaybackHandlers;
  private durationMs = 0;
  private atMs = 0;
  private playing = false;
  private frame: number | null = null;
  private lastFrame = 0;

  constructor(handlers: PlaybackHandlers) {
    this.handlers = handlers;
  }

  at(): number {
    return this.atMs;
  }

  duration(): number {
    return this.durationMs;
  }

  isPlaying(): boolean {
    return this.playing;
  }

  /** The run changed length. Stay where we are, within the new run. */
  setDuration(ms: number) {
    this.durationMs = Math.max(0, ms);
    this.seek(Math.min(this.atMs, this.durationMs));
  }

  seek(ms: number) {
    this.atMs = Math.min(this.durationMs, Math.max(0, Math.round(ms)));
    this.handlers.onTick(this.atMs, this.playing);
  }

  play() {
    if (this.playing || this.durationMs <= 0) {
      return;
    }

    // pressing play at the end starts again, rather than doing nothing
    if (this.atMs >= this.durationMs) {
      this.atMs = 0;
    }

    this.playing = true;
    this.lastFrame = performance.now();
    this.frame = requestAnimationFrame(this.advance);

    this.handlers.onTick(this.atMs, true);
  }

  pause() {
    if (!this.playing) {
      return;
    }

    this.playing = false;

    if (this.frame !== null) {
      cancelAnimationFrame(this.frame);
      this.frame = null;
    }

    this.handlers.onTick(this.atMs, false);
  }

  toggle() {
    if (this.playing) {
      this.pause();
    } else {
      this.play();
    }
  }

  private advance = (now: number) => {
    if (!this.playing) {
      return;
    }

    // real time, so the run on screen takes as long as the run on the mat
    this.atMs += now - this.lastFrame;
    this.lastFrame = now;

    if (this.atMs >= this.durationMs) {
      this.atMs = this.durationMs;
      this.playing = false;
      this.frame = null;
      this.handlers.onTick(this.atMs, false);
      return;
    }

    this.handlers.onTick(this.atMs, true);
    this.frame = requestAnimationFrame(this.advance);
  };
}
