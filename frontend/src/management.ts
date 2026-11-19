/** An explicit initialization grants a page-local lease; reloads never resume it. */
export class ManagementLease {
  token: string | null = null
  established = false
  grant(token: string) { this.token = token; this.established = false }
  clear() { this.token = null; this.established = false }
  observe(fresh: boolean, enabled: boolean) {
    if (!fresh || this.established && !enabled) this.clear()
    else if (enabled && this.token) this.established = true
  }
  payload() { return this.token ? { manager_token: this.token } : null }
  message() { return this.token ? { type: 'heartbeat', manager_token: this.token } : null }
}
