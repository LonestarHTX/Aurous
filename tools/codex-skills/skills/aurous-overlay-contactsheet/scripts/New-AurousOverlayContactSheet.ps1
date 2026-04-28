param(
	[Parameter(Mandatory = $true)][string]$ExportDir,
	[string]$OutputPath = '',
	[string[]]$Layers = @(
		'PlateId',
		'BoundaryMask',
		'ContinentalWeight',
		'MaterialOwnerMismatch',
		'MaterialOverlap',
		'DivergentBoundary',
		'OceanCrustAge',
		'OceanCrustElevation',
		'OceanCrustThickness',
		'OceanCrustId',
		'CrustEventOverlay'
	),
	[int]$Columns = 3,
	[int]$ThumbnailWidth = 420
)

$ErrorActionPreference = 'Stop'

function ConvertTo-SafeName {
	param([string]$Value)
	return ($Value -replace '[^A-Za-z0-9_.-]+', '-').Trim('-')
}

function Find-LayerImage {
	param([string]$Root, [string]$Layer)
	$Candidates = @(
		Get-ChildItem -LiteralPath $Root -Recurse -File -Filter '*.png' -ErrorAction SilentlyContinue |
			Where-Object { $_.BaseName -eq $Layer -or $_.BaseName -like "$Layer*" } |
			Sort-Object -Property @{ Expression = 'LastWriteTime'; Descending = $true }, FullName
	)
	return $Candidates | Select-Object -First 1
}

if (-not (Test-Path -LiteralPath $ExportDir -PathType Container)) {
	throw "ExportDir is not a directory: $ExportDir"
}

$ResolvedExportDir = (Resolve-Path -LiteralPath $ExportDir).Path
if ([string]::IsNullOrWhiteSpace($OutputPath)) {
	$RepoRoot = if ($ResolvedExportDir -match '^(.*?\\Aurous)\\') { $Matches[1] } else { (Get-Location).Path }
	$OutputDir = Join-Path $RepoRoot 'Saved\Diagnostics'
	$OutputPath = Join-Path $OutputDir ('overlay-contactsheet-' + (ConvertTo-SafeName (Split-Path -Leaf $ResolvedExportDir)) + '.png')
}

$Found = foreach ($Layer in $Layers) {
	$Image = Find-LayerImage -Root $ResolvedExportDir -Layer $Layer
	if ($null -ne $Image) {
		[pscustomobject]@{
			layer = $Layer
			path = $Image.FullName
			name = $Image.Name
			last_write_time = $Image.LastWriteTime
		}
	}
}
$Missing = @($Layers | Where-Object { $Layer = $_; -not ($Found | Where-Object { $_.layer -eq $Layer }) })

if ($Found.Count -eq 0) {
	[pscustomobject]@{
		status = 'no_matching_images'
		export_dir = $ResolvedExportDir
		output_path = $OutputPath
		missing_layers = $Missing
	} | ConvertTo-Json -Depth 5
	exit 1
}

Add-Type -AssemblyName System.Drawing

$Rows = [Math]::Ceiling($Found.Count / [double]$Columns)
$TitleHeight = 30
$Padding = 14
$CellImageHeight = [int]($ThumbnailWidth * 0.62)
$CellWidth = $ThumbnailWidth + ($Padding * 2)
$CellHeight = $CellImageHeight + $TitleHeight + ($Padding * 2)
$SheetWidth = $CellWidth * $Columns
$SheetHeight = [int]($CellHeight * $Rows)

New-Item -ItemType Directory -Force -Path (Split-Path -Parent $OutputPath) | Out-Null

$Bitmap = [System.Drawing.Bitmap]::new($SheetWidth, $SheetHeight)
$Graphics = [System.Drawing.Graphics]::FromImage($Bitmap)
$TitleFont = [System.Drawing.Font]::new('Segoe UI', 11, [System.Drawing.FontStyle]::Bold)
$SubFont = [System.Drawing.Font]::new('Segoe UI', 8)
$White = [System.Drawing.SolidBrush]::new([System.Drawing.Color]::White)
$Ink = [System.Drawing.SolidBrush]::new([System.Drawing.Color]::FromArgb(25, 25, 25))
$Muted = [System.Drawing.SolidBrush]::new([System.Drawing.Color]::FromArgb(90, 90, 90))
$BorderPen = [System.Drawing.Pen]::new([System.Drawing.Color]::FromArgb(190, 190, 190), 1)

try {
	$Graphics.Clear([System.Drawing.Color]::White)
	for ($Index = 0; $Index -lt $Found.Count; $Index++) {
		$Item = $Found[$Index]
		$Column = $Index % $Columns
		$Row = [Math]::Floor($Index / $Columns)
		$X = $Column * $CellWidth
		$Y = $Row * $CellHeight
		$Graphics.DrawRectangle($BorderPen, $X + 4, $Y + 4, $CellWidth - 8, $CellHeight - 8)
		$Graphics.DrawString($Item.layer, $TitleFont, $Ink, $X + $Padding, $Y + 8)
		$Graphics.DrawString($Item.name, $SubFont, $Muted, $X + $Padding, $Y + 28)

		$Image = [System.Drawing.Image]::FromFile($Item.path)
		try {
			$MaxW = $ThumbnailWidth
			$MaxH = $CellImageHeight
			$Scale = [Math]::Min($MaxW / [double]$Image.Width, $MaxH / [double]$Image.Height)
			$DrawW = [int]($Image.Width * $Scale)
			$DrawH = [int]($Image.Height * $Scale)
			$DrawX = $X + $Padding + [int](($MaxW - $DrawW) / 2)
			$DrawY = $Y + $TitleHeight + $Padding + [int](($MaxH - $DrawH) / 2)
			$Graphics.DrawImage($Image, $DrawX, $DrawY, $DrawW, $DrawH)
		} finally {
			$Image.Dispose()
		}
	}
	$Bitmap.Save($OutputPath, [System.Drawing.Imaging.ImageFormat]::Png)
} finally {
	$BorderPen.Dispose()
	$Muted.Dispose()
	$Ink.Dispose()
	$White.Dispose()
	$SubFont.Dispose()
	$TitleFont.Dispose()
	$Graphics.Dispose()
	$Bitmap.Dispose()
}

[pscustomobject]@{
	status = 'ok'
	export_dir = $ResolvedExportDir
	output_path = (Resolve-Path -LiteralPath $OutputPath).Path
	included_layers = $Found
	missing_layers = $Missing
	columns = $Columns
	thumbnail_width = $ThumbnailWidth
} | ConvertTo-Json -Depth 6
