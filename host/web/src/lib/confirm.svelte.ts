// Promise-based confirmation dialog: `if (await confirm({...})) ...`.

export interface ConfirmRequest {
  title: string;
  body: string;
  confirmText?: string;
  danger?: boolean;
}

class ConfirmState {
  request = $state<ConfirmRequest | null>(null);
  private resolve: ((ok: boolean) => void) | null = null;

  ask(request: ConfirmRequest): Promise<boolean> {
    this.resolve?.(false);
    this.request = request;
    return new Promise((resolve) => (this.resolve = resolve));
  }

  answer(ok: boolean): void {
    this.resolve?.(ok);
    this.resolve = null;
    this.request = null;
  }
}

export const confirmState = new ConfirmState();
export const confirm = (request: ConfirmRequest) => confirmState.ask(request);
