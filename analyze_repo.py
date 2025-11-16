#!/usr/bin/env python3
"""
Script d'analyse de repository GitHub
Génère un fichier .txt avec la structure complète du projet
"""

import os
import subprocess
# from pathlib import Path  # Unused
from datetime import datetime

def get_repo_info(repo_path):
    """Récupère les infos du repo"""
    os.chdir(repo_path)
    
    try:
        remote_url = subprocess.check_output(['git', 'config', '--get', 'remote.origin.url']).decode().strip()
    except:
        remote_url = "N/A"
    
    try:
        current_branch = subprocess.check_output(['git', 'rev-parse', '--abbrev-ref', 'HEAD']).decode().strip()
    except:
        current_branch = "N/A"
    
    try:
        commit_count = subprocess.check_output(['git', 'rev-list', '--count', 'HEAD']).decode().strip()
    except:
        commit_count = "N/A"
    
    try:
        last_commit = subprocess.check_output(['git', 'log', '-1', '--format=%ai']).decode().strip()
    except:
        last_commit = "N/A"
    
    return {
        'remote': remote_url,
        'branch': current_branch,
        'commits': commit_count,
        'last_commit': last_commit
    }

def get_file_tree(repo_path, prefix="", max_depth=5, current_depth=0, ignore_dirs={'.git', '__pycache__', '.pytest_cache', 'node_modules', '.venv', 'venv'}):
    """Génère un arbre des fichiers"""
    if current_depth >= max_depth:
        return []
    
    lines = []
    try:
        items = sorted(os.listdir(repo_path))
    except PermissionError:
        return lines
    
    # Filtrer les dossiers à ignorer
    items = [item for item in items if item not in ignore_dirs]
    
    for i, item in enumerate(items):
        path = os.path.join(repo_path, item)
        is_last = i == len(items) - 1
        current_prefix = "└── " if is_last else "├── "
        lines.append(f"{prefix}{current_prefix}{item}")
        
        if os.path.isdir(path) and not item.startswith('.'):
            next_prefix = prefix + ("    " if is_last else "│   ")
            lines.extend(get_file_tree(path, next_prefix, max_depth, current_depth + 1, ignore_dirs))
    
    return lines

def count_lines_of_code(repo_path):
    """Compte les lignes de code par type de fichier"""
    stats = {}
    
    for root, dirs, files in os.walk(repo_path):
        # Ignorer les dossiers
        dirs[:] = [d for d in dirs if d not in {'.git', '__pycache__', '.pytest_cache', 'node_modules', '.venv', 'venv'}]
        
        for file in files:
            ext = os.path.splitext(file)[1] or 'no_extension'
            filepath = os.path.join(root, file)
            
            try:
                with open(filepath, 'r', encoding='utf-8', errors='ignore') as f:
                    lines = len(f.readlines())
                    if ext not in stats:
                        stats[ext] = {'files': 0, 'lines': 0}
                    stats[ext]['files'] += 1
                    stats[ext]['lines'] += lines
            except:
                pass
    
    return stats

def get_main_files(repo_path):
    """Récupère les fichiers principaux (README, requirements, etc)"""
    important_files = ['README.md', 'requirements.txt', 'setup.py', 'package.json', 'Dockerfile', 'CMakeLists.txt', '.github', 'LICENSE']
    found_files = []
    
    for item in important_files:
        path = os.path.join(repo_path, item)
        if os.path.exists(path):
            if os.path.isfile(path):
                found_files.append(item)
            else:
                found_files.append(f"{item}/ (directory)")
    
    return found_files

def analyze_python_modules(repo_path):
    """Trouve les modules Python principaux"""
    modules = []
    
    for root, dirs, files in os.walk(repo_path):
        dirs[:] = [d for d in dirs if d not in {'.git', '__pycache__', '.pytest_cache', '.venv', 'venv'}]
        
        for file in files:
            if file.endswith('.py') and not file.startswith('__'):
                rel_path = os.path.relpath(os.path.join(root, file), repo_path)
                if not rel_path.startswith('.'):
                    modules.append(rel_path)
    
    return sorted(modules)[:30]  # Top 30

def generate_analysis(repo_path, output_file):
    """Génère l'analyse complète"""
    
    print(f"🔍 Analysing repository: {repo_path}")
    
    output = []
    output.append("=" * 80)
    output.append("GITHUB REPOSITORY ANALYSIS")
    output.append("=" * 80)
    output.append(f"\nGenerated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # Info du repo
    print("📊 Fetching repository info...")
    info = get_repo_info(repo_path)
    output.append("\n" + "=" * 80)
    output.append("REPOSITORY INFO")
    output.append("=" * 80)
    output.append(f"Remote URL: {info['remote']}")
    output.append(f"Current Branch: {info['branch']}")
    output.append(f"Total Commits: {info['commits']}")
    output.append(f"Last Commit: {info['last_commit']}")
    
    # Fichiers importants
    print("📁 Finding important files...")
    important = get_main_files(repo_path)
    if important:
        output.append("\n" + "=" * 80)
        output.append("IMPORTANT FILES")
        output.append("=" * 80)
        for f in important:
            output.append(f"  • {f}")
    
    # Stats de code
    print("📈 Counting lines of code...")
    stats = count_lines_of_code(repo_path)
    output.append("\n" + "=" * 80)
    output.append("CODE STATISTICS")
    output.append("=" * 80)
    total_lines = sum(s['lines'] for s in stats.values())
    total_files = sum(s['files'] for s in stats.values())
    output.append(f"Total Files: {total_files}")
    output.append(f"Total Lines of Code: {total_lines}\n")
    
    output.append("Breakdown by file type:")
    for ext in sorted(stats.keys(), key=lambda x: stats[x]['lines'], reverse=True):
        s = stats[ext]
        output.append(f"  {ext:15} → {s['files']:3} files, {s['lines']:6} lines")
    
    # Modules Python
    print("🐍 Finding Python modules...")
    python_modules = analyze_python_modules(repo_path)
    if python_modules:
        output.append("\n" + "=" * 80)
        output.append("PYTHON MODULES (Top 30)")
        output.append("=" * 80)
        for module in python_modules:
            output.append(f"  • {module}")
    
    # Arbre des fichiers
    print("🌳 Building file tree...")
    output.append("\n" + "=" * 80)
    output.append("DIRECTORY STRUCTURE")
    output.append("=" * 80)
    tree = get_file_tree(repo_path)
    output.extend(tree[:200])  # Limiter à 200 lignes
    if len(tree) > 200:
        output.append(f"  ... and {len(tree) - 200} more items")
    
    output.append("\n" + "=" * 80)
    output.append("END OF ANALYSIS")
    output.append("=" * 80)
    
    # Écrire le fichier
    print(f"✅ Writing to {output_file}")
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write('\n'.join(output))
    
    print(f"✅ Analysis complete! File saved to: {output_file}")
    print(f"📊 Total: {total_files} files, {total_lines} lines of code")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        repo_path = sys.argv[1]
    else:
        repo_path = os.getcwd()
    
    if not os.path.exists(os.path.join(repo_path, '.git')):
        print("❌ Error: Not a Git repository!")
        sys.exit(1)
    
    output_file = os.path.join(repo_path, "REPOSITORY_ANALYSIS.txt")
    generate_analysis(repo_path, output_file)